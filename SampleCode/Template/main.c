/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#define ENABLE_CAP_LOG
//#define ENABLE_ADC_LOG

#define PLL_CLOCK           		192000000

#define LED_R					(PH0)
#define LED_Y					(PH1)
#define LED_G					(PH2)

#define BUF_LEN					(1024)

#define FIFO_THRESHOLD 			(4)
#define RX_BUFFER_SIZE 			(256)
#define RX_TIMEOUT_CNT 			(60) //40~255

#define UART_RX_IDEL(uart) (((uart)->FIFOSTS & UART_FIFOSTS_RXIDLE_Msk )>> UART_FIFOSTS_RXIDLE_Pos)

typedef struct {
	uint8_t RX_Buffer[RX_BUFFER_SIZE];
	uint16_t Length;
	uint8_t RDA_Trigger_Cnt;
	uint8_t RXTO_Trigger_Cnt;
	
//	uint8_t end;
}UART_BUF_t;

UART_BUF_t uart0Dev;
UART_BUF_t uart2Dev;

typedef enum{
	flag_DEFAULT = 0 ,
		
	flag_UART0_Received_Data ,	
	flag_UART2_Received_Data ,

	flag_Trans_Data_Ready , 	
	flag_ADC_Data_Ready ,

	
	flag_END	
}Flag_Index;

/*****************************************************************************/
volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))
/*****************************************************************************/

enum
{
	ADC0_CH0 = 0 ,
	ADC0_CH1 ,
	ADC0_CH2 , 
	ADC0_CH3 , 
	ADC0_CH4 ,
	ADC0_CH5 , 
	ADC0_CH6 , 
	ADC0_CH7 ,
	ADC0_CH8 , 
	ADC0_CH9 , 
	ADC0_CH10 , 
	ADC0_CH11 ,
	ADC0_CH12 , 
	ADC0_CH13 , 
	ADC0_CH14 ,
	ADC0_CH15 , 
	
	ADC0_CH16_BAND_GAP_VOLT , 
	ADC0_CH17_TEMP_SENSOR ,
	ADC0_CH18_VBAT , 
	
	ADC_CH_DEFAULT 	
}ADC_CH_TypeDef;

#define ADC_DIGITAL_SCALE(void) 					(0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 
#define ADC_CALC_DATA_TO_VOLTAGE(DATA,VREF) ((DATA) * (VREF) / ADC_DIGITAL_SCALE())

#define PDMA_CH_ADC 							(15)
#define ADC_PDMA_OPENED_CH   					(1 << PDMA_CH_ADC)
#define ADC_NUM_COUNT 						(4)

uint16_t aADCIRQConvertedData[ADC_NUM_COUNT] = {0};
uint16_t pdmaConvertedData[ADC_NUM_COUNT] = {0};

#define PDMA_CH_CAP  							(14)
#define CAPx_PDMA_OPENED_CH   				(1 << PDMA_CH_CAP)
#define EPWM_CAPx_CH							(0)


__IO uint16_t g_u32Count[4];
volatile uint32_t g_u32CH0TestOver = 0;

uint16_t capt_psc = 0;
uint32_t capt_div_freq = 0 ;



/*
	Target : 30K Freq
	DUTY : 50%
	
	SYS_CLK : 192M
	PSC : 1

	192 000 000/30 000 = PSC * (CNR + 1)
	CNR = (SYS_CLK/FREQ)/PSC - 1 = 6400 - 1

	DUTY ratio = CMR / (CNR + 1)
	50% = CMR / (CNR + 1)
	
*/

#define SYS_CLK 									(PLL_CLOCK)
#define PWM_PSC 								(1)	
#define PWM_FREQ 								(30000)	
#define PWM_TARGET_DUTY(d)					((PWM_RESOLUTION*(100-d))/100)	//((PWM_RESOLUTION*d)/100)

#define PWM_DUTY                              	(PWM_TARGET_DUTY(50))		//percent
#define PWM_RESOLUTION                        	(0x1000)

#define PWM_CHANNEL                           	(4)
#define PWM_CHANNEL_MASK                     (EPWM_CH_4_MASK)

//16 bit
/*
	Up-Down Counter Type : 
	EPWM period time =(2*PERIOD) * (CLKPSC+1) * EPWMx_CLK. 

	Up Counter Type  or Down Counter Type 
	EPWM period time = (PERIOD+1) *(CLKPSC+1)* EPWMx_CLK. 
	
*/

//#define PWM_CNR 								((SYS_CLK/PWM_FREQ)/PWM_PSC - 1)			//Up Counter Type  or Down Counter Type
#define PWM_CNR 								(((SYS_CLK/PWM_FREQ)/PWM_PSC - 1)>>1 )	//Up-Down Counter Type
#define PWM_CMR 								(PWM_DUTY * (PWM_CNR + 1)/PWM_RESOLUTION)

#define CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)	(u32DutyCycle * (EPWM_GET_CNR(pwm, u32ChannelNum) + 1) / u32CycleResolution)
#define CalNewDuty(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)		(EPWM_SET_CMR(pwm, u32ChannelNum, CalNewDutyCMR(pwm, u32ChannelNum, u32DutyCycle, u32CycleResolution)))

/*****************************************************************************/

void CalPeriodTime(EPWM_T *EPWM, uint32_t u32Ch)
{
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
	uint16_t u16Duty = 0;	
	uint32_t u32Freq = 0;

    g_u32CH0TestOver = 0;
    /* Wait PDMA interrupt (g_u32CH0TestOver will be set at IRQ_Handler function) */
    while(g_u32CH0TestOver == 0);

    u16RisingTime = g_u32Count[1];

    u16FallingTime = g_u32Count[0];

    u16HighPeriod = g_u32Count[1] - g_u32Count[2] + 1;

    u16LowPeriod = 0xFFFF - g_u32Count[1];

    u16TotalPeriod = 0xFFFF - g_u32Count[2] + 1;

	u16Duty = u16HighPeriod*100/u16TotalPeriod;
	
	u32Freq = capt_div_freq/u16TotalPeriod;	//Up Counter Type  or Down Counter Type
	
	#if defined (ENABLE_CAP_LOG)	//debug
    printf("Rising=%5d,Falling=%5d,High=%5d,Low=%5d,Total=%5d,Freq=%5d,Duty=%3d.\r\n",
           u16RisingTime, 
           u16FallingTime, 
           u16HighPeriod, 
           u16LowPeriod, 
           u16TotalPeriod,
           u32Freq,
           u16Duty);
	#endif
}


void CaptureCalculation(void)
{
    EPWM1->CAPCTL |= EPWM_CAPCTL_FCRLDEN0_Msk;	
	
//    while((EPWM1->CNT[EPWM_CAPx_CH]) == 0);

    /* Capture the Input Waveform Data */
    CalPeriodTime(EPWM1, EPWM_CAPx_CH);
	
}

void EPWM_CAP_ReloadPDMA(void)
{
    PDMA_SetTransferCnt(PDMA,PDMA_CH_CAP, PDMA_WIDTH_16, 4);
    PDMA_SetTransferMode(PDMA,PDMA_CH_CAP, PDMA_EPWM1_P1_RX, FALSE, 0);
}

void EPWM_CAP_PDMA_Init(void)
{
    PDMA_Open(PDMA,BIT0 << PDMA_CH_CAP);

    PDMA_SetTransferCnt(PDMA,PDMA_CH_CAP, PDMA_WIDTH_16, 4);

    PDMA_SetTransferAddr(PDMA,PDMA_CH_CAP, (uint32_t)&(EPWM1->PDMACAP[0]), PDMA_SAR_FIX, (uint32_t)&g_u32Count[0], PDMA_DAR_INC);

    PDMA_SetTransferMode(PDMA,PDMA_CH_CAP, PDMA_EPWM1_P1_RX, FALSE, 0);

    PDMA_SetBurstType(PDMA,PDMA_CH_CAP, PDMA_REQ_SINGLE, PDMA_BURST_4);

    PDMA_EnableInt(PDMA,PDMA_CH_CAP, PDMA_INT_TRANS_DONE);
	
    NVIC_EnableIRQ(PDMA_IRQn);

    EPWM_EnablePDMA(EPWM1, EPWM_CAPx_CH, FALSE, EPWM_CAPTURE_PDMA_RISING_FALLING_LATCH);
}

void EPWM_CAP_Init(void)	
{
	uint32_t target_ns = 0;
		
	/*
			use PLL : 192000000
			use PCLK1 : depend on div. , exmaple : div = 2 , capture clock = 192000000/2 = 96000000
	
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency

           how to get PSC , make CNR+1 close to 0xFFFF , 
           target freq = 100  
           ==> CNR+1 = 192000000 / PSC / min_freq , when CNR = 65535 with min freq = 100 , will get PSC 30

           Capture unit time = 1/Capture clock source frequency/prescaler
           ==> target ns = 1/ 192000000 / 30 = 30/ 192000000 = 156 ns

	*/
	capt_psc = 30;
	target_ns = 156;
	capt_div_freq = FREQ_192MHZ/capt_psc;

    EPWM_ConfigCaptureChannel(EPWM1, EPWM_CAPx_CH, target_ns, 0);	//target_ns

    EPWM_Start(EPWM1, EPWM_CH_0_MASK);

    EPWM_EnableCapture(EPWM1, EPWM_CH_0_MASK);

//    EPWM1->CAPCTL |= EPWM_CAPCTL_FCRLDEN5_Msk;

//    while((EPWM1->CNT[EPWM_CAPx_CH]) == 0);

//    CalPeriodTime(EPWM1, EPWM_CAPx_CH);
}


void EPWM_Init(void)	//EPWM0_CH4 : PA1 , 30K , resolution 12 bit
{
    /* Set PWM0 timer clock prescaler */
    EPWM_SET_PRESCALER(EPWM0, PWM_CHANNEL, PWM_PSC - 1);
	
    /* Set up counter type */
    EPWM0->CTL1 &= ~EPWM_CTL1_CNTTYPE4_Msk;

    /* Set PWM0 timer period */
    EPWM_SET_CNR(EPWM0, PWM_CHANNEL, PWM_CNR);
	
    /* Set PWM0 timer duty */
    EPWM_SET_CMR(EPWM0, PWM_CHANNEL, PWM_CMR);	

    EPWM_EnableADCTrigger(EPWM0, PWM_CHANNEL , EPWM_TRG_ADC_EVEN_PERIOD);

	EPWM_SET_ALIGNED_TYPE(EPWM0, PWM_CHANNEL_MASK, EPWM_CENTER_ALIGNED);
	
    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
//    EPWM_SET_OUTPUT_LEVEL(EPWM0, PWM_CHANNEL_MASK, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_LOW, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_NOTHING);
    EPWM_SET_OUTPUT_LEVEL(EPWM0, PWM_CHANNEL_MASK, EPWM_OUTPUT_LOW, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_LOW);
	
    /* Enable output of PWM0 channel 0 */
    EPWM_EnableOutput(EPWM0, PWM_CHANNEL_MASK);
	
	EPWM_Start(EPWM0, PWM_CHANNEL_MASK);
	
}


void EADC_ReloadPDMA(void)
{
    /* transfer width is half word(16 bit) and transfer count is ADCDatalenght+1 */
    PDMA_SetTransferCnt(PDMA, PDMA_CH_ADC, PDMA_WIDTH_16, ADC_NUM_COUNT);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, PDMA_CH_ADC, PDMA_EADC0_RX, FALSE, (uint32_t) 0);
}


void EADC_PDMA_Init(void)
{
	SYS_ResetModule(PDMA_RST);

    /* Configure PDMA peripheral mode form ADC to memory */
    /* Open PDMA Channel 1 based on PDMA_CH_ADC setting*/
    PDMA_Open(PDMA, BIT0 << PDMA_CH_ADC);

    /* transfer width is half word(16 bit) and transfer count is ADCDatalenght+1 */
    PDMA_SetTransferCnt(PDMA, PDMA_CH_ADC, PDMA_WIDTH_16, ADC_NUM_COUNT);

    /* Set source address as ADC data register (no increment) and destination address as g_i32ConversionData array (increment) */
    PDMA_SetTransferAddr(PDMA, PDMA_CH_ADC, (uint32_t)& (EADC->CURDAT), PDMA_SAR_FIX, (uint32_t)pdmaConvertedData, PDMA_DAR_INC);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, PDMA_CH_ADC, PDMA_EADC0_RX, FALSE, 0);

    /* Set PDMA as single request type for ADC */
    PDMA_SetBurstType(PDMA, PDMA_CH_ADC, PDMA_REQ_SINGLE, PDMA_BURST_128);

    PDMA_EnableInt(PDMA, PDMA_CH_ADC, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);
	
	PDMA_Trigger(PDMA , PDMA_CH_ADC);

    /* ADC enable PDMA transfer */
    EADC_ENABLE_PDMA(EADC);
}


void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);
	
    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
		#if 0
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
		#else
        if (PDMA_GET_ABORT_STS(PDMA) & ADC_PDMA_OPENED_CH)
        {
			printf("ABTSTS\r\n");
        }
        PDMA_CLR_ABORT_FLAG(PDMA, ADC_PDMA_OPENED_CH);

        if(PDMA_GET_ABORT_STS(PDMA) & CAPx_PDMA_OPENED_CH)
            g_u32CH0TestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA,CAPx_PDMA_OPENED_CH);
		

		#endif
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if(PDMA_GET_TD_STS(PDMA) & ADC_PDMA_OPENED_CH)
        {
			//insert process
//			set_flag(flag_Trans_Data_Ready , ENABLE);
//			LED_G ^= 1;
			EADC_ReloadPDMA();

        }        

		/* Clear PDMA transfer done interrupt flag */
		PDMA_CLR_TD_FLAG(PDMA, ADC_PDMA_OPENED_CH);

        if(PDMA_GET_TD_STS(PDMA) & CAPx_PDMA_OPENED_CH)
        {		
			g_u32CH0TestOver = 1;			

			EPWM_CAP_ReloadPDMA();
//			EPWM_CAP_PDMA_Init();
//			EPWM_CAP_Init();
        }
		PDMA_CLR_TD_FLAG(PDMA,CAPx_PDMA_OPENED_CH);
		
    }
    else if (status & (PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
    {
        PDMA_CLR_TMOUT_FLAG(PDMA,PDMA_CH_ADC);
		printf("REQTOF\r\n");

    }
    else
    {

    }
	
}

void EADC00_IRQHandler(void)
{
	uint8_t ModulePriority = ADC0_CH5;

	aADCIRQConvertedData[0] = EADC_GET_CONV_DATA(EADC, ModulePriority);
	LED_R ^= 1;
	
//    set_flag(flag_ADC_Band_GAP , ENABLE);
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

void EADC01_IRQHandler(void)
{
	uint8_t ModulePriority = ADC0_CH5;

	aADCIRQConvertedData[1] = EADC_GET_CONV_DATA(EADC, ModulePriority+1);
	aADCIRQConvertedData[2] = EADC_GET_CONV_DATA(EADC, ModulePriority+2);
	aADCIRQConvertedData[3] = EADC_GET_CONV_DATA(EADC, ModulePriority+3);

//    set_flag(flag_ADC_Data_Ready , ENABLE);
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);
}

void ADC_Convert_Init(void)
{
	uint8_t ModulePriority = ADC0_CH5;
	
//	set_flag(flag_Trans_Data_Ready , DISABLE);	
//	set_flag(flag_ADC_Data_Ready , DISABLE);
	
    /* Set input mode as single-end, and Single mode*/
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

	EADC_ConfigSampleModule(EADC, ModulePriority, EADC_EPWM0TG4_TRIGGER, ADC0_CH5);
	EADC_ConfigSampleModule(EADC, ModulePriority+1, EADC_ADINT0_TRIGGER, ADC0_CH0);
	EADC_ConfigSampleModule(EADC, ModulePriority+2, EADC_ADINT0_TRIGGER, ADC0_CH1);
	EADC_ConfigSampleModule(EADC, ModulePriority+3, EADC_ADINT0_TRIGGER, ADC0_CH2);

    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);
    EADC_ENABLE_INT(EADC, BIT0);
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, (BIT0 << ModulePriority));

    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);
    EADC_ENABLE_INT(EADC, BIT1);
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 1, (BIT0 << (ModulePriority+3)));

    NVIC_EnableIRQ(EADC00_IRQn);
    NVIC_EnableIRQ(EADC01_IRQn);
	
	EADC_PDMA_Init();

    EADC_START_CONV(EADC, (BIT0 << ADC0_CH5));
}

void EADC_Convert_Result(void)
{
	uint8_t i = 0;

	#if defined (ENABLE_ADC_LOG)	//debug
	for (i = 0 ; i < ADC_NUM_COUNT; i++)
	{
		printf("0x%3X,0x%3X,|" , aADCIRQConvertedData[i],pdmaConvertedData[i] );
	}

	printf("\r\n");
	#endif
	
}


void TMR0_IRQHandler(void)
{
	static uint8_t flag = 0;
	static uint16_t CNT_flag = 0;
	
//	static uint32_t LOG = 0;
	static uint16_t CNT = 0;

    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

		if (CNT++ >= 1000)
		{
			CNT = 0;
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);

			LED_G ^= 1;	
		}

		if (CNT_flag++ >= 5000)
		{
			CNT_flag = 0;
			flag = (flag >=3) ? (0) : (flag+1) ;
		}

		switch(flag)
		{
			case 0:
				CalNewDuty(EPWM0, PWM_CHANNEL, PWM_TARGET_DUTY(50), PWM_RESOLUTION);				
				break;			
			case 1:
				CalNewDuty(EPWM0, PWM_CHANNEL, PWM_TARGET_DUTY(85), PWM_RESOLUTION);				
				break;
			case 2:
				CalNewDuty(EPWM0, PWM_CHANNEL, PWM_TARGET_DUTY(38.5), PWM_RESOLUTION);				
				break;
			case 3:
				CalNewDuty(EPWM0, PWM_CHANNEL, PWM_TARGET_DUTY(62.7), PWM_RESOLUTION);	
				break;					
		}		
    }
}

void TIMER0_Init(void)
{
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);	
    TIMER_Start(TIMER0);
}

void GPIO_Init(void)
{
    GPIO_SetMode(PH, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT2, GPIO_MODE_OUTPUT);	
}


void UART2_IRQHandler(void)
{
	uint8_t i;
	static uint16_t u16UART_RX_Buffer_Index = 0;

    if(UART_GET_INT_FLAG(UART2, UART_INTSTS_RDAINT_Msk))    
    {
        /* UART receive data available flag */
        
        /* Record RDA interrupt trigger times */
        uart2Dev.RDA_Trigger_Cnt++;
        
        /* Move the data from Rx FIFO to sw buffer (RAM). */
        /* Every time leave 1 byte data in FIFO for Rx timeout */
        for(i = 0 ; i < (FIFO_THRESHOLD - 1) ; i++)
        {
            uart2Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART2);
            u16UART_RX_Buffer_Index ++;

            if (u16UART_RX_Buffer_Index >= RX_BUFFER_SIZE) 
                u16UART_RX_Buffer_Index = 0;
        }
    }
    else if(UART_GET_INT_FLAG(UART2, UART_INTSTS_RXTOINT_Msk)) 
    {
        /* When Rx timeout flag is set to 1, it means there is no data needs to be transmitted. */

        /* Record Timeout times */
        uart2Dev.RXTO_Trigger_Cnt++;

        /* Move the last data from Rx FIFO to sw buffer. */
        while(UART_GET_RX_EMPTY(UART2) == 0)
        {
            uart2Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART2);
            u16UART_RX_Buffer_Index ++;
        }

        /* Clear UART RX parameter */
        UART_DISABLE_INT(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
        u16UART_RX_Buffer_Index = 0;

//		set_flag(flag_UART2_Received_Data , ENABLE);

        printf("\nUART2 Rx Received Data : %s\n",uart2Dev.RX_Buffer);
        printf("UART2 Rx RDA (Fifofull) interrupt times : %d\n",uart2Dev.RDA_Trigger_Cnt);
        printf("UART2 Rx RXTO (Timeout) interrupt times : %d\n",uart2Dev.RXTO_Trigger_Cnt);

        /* Reset UART interrupt parameter */
        UART_EnableInt(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
		memset(&uart2Dev, 0x00, sizeof(UART_BUF_t));

    }
	
}


void UART2_Init(void)
{
    SYS_ResetModule(UART2_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART2, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART2, RX_TIMEOUT_CNT);

    UART2->FIFO = ((UART2->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES);

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART2, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);	
	NVIC_EnableIRQ(UART2_IRQn);

	memset(&uart2Dev, 0x00, sizeof(UART_BUF_t));
	
	UART_WAIT_TX_EMPTY(UART2);

//	set_flag(flag_UART2_Received_Data , DISABLE);
	
}

void UART0_IRQHandler(void)
{
	uint8_t i;
	static uint16_t u16UART_RX_Buffer_Index = 0;

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk))    
    {
        /* UART receive data available flag */
        
        /* Record RDA interrupt trigger times */
        uart0Dev.RDA_Trigger_Cnt++;
        
        /* Move the data from Rx FIFO to sw buffer (RAM). */
        /* Every time leave 1 byte data in FIFO for Rx timeout */
        for(i = 0 ; i < (FIFO_THRESHOLD - 1) ; i++)
        {
            uart0Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;

            if (u16UART_RX_Buffer_Index >= RX_BUFFER_SIZE) 
                u16UART_RX_Buffer_Index = 0;
        }
    }
    else if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RXTOINT_Msk)) 
    {
        /* When Rx timeout flag is set to 1, it means there is no data needs to be transmitted. */

        /* Record Timeout times */
        uart0Dev.RXTO_Trigger_Cnt++;

        /* Move the last data from Rx FIFO to sw buffer. */
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            uart0Dev.RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;
        }

        /* Clear UART RX parameter */
        UART_DISABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
        u16UART_RX_Buffer_Index = 0;

//		set_flag(flag_UART0_Received_Data , ENABLE);

        printf("\nUART0 Rx Received Data : %s\n",uart0Dev.RX_Buffer);
        printf("UART0 Rx RDA (Fifofull) interrupt times : %d\n",uart0Dev.RDA_Trigger_Cnt);
        printf("UART0 Rx RXTO (Timeout) interrupt times : %d\n",uart0Dev.RXTO_Trigger_Cnt);

        /* Reset UART interrupt parameter */
        UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
		memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));

    }
	
}

void UART0_Init(void)
{
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, RX_TIMEOUT_CNT);

	/* Set UART FIFO RX interrupt trigger level to 4-bytes*/
    UART0->FIFO = ((UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES);

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);
	
	memset(&uart0Dev, 0x00, sizeof(UART_BUF_t));

	UART_WAIT_TX_EMPTY(UART0);
	
//	set_flag(flag_UART0_Received_Data , DISABLE);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART2_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HXT, CLK_CLKDIV4_UART2(1));

    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);


    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is 96MHz, set divider to 8, EADC clock is 96/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable IP module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* EPWM clock frequency is set double to PCLK: select EPWM module clock source as PLL */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PLL, (uint32_t)NULL);

   	SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA1MFP_Msk );
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA1MFP_EPWM0_CH4 );	

    CLK_EnableModuleClock(EPWM1_MODULE);

    CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PLL, (uint32_t)NULL);

    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk );
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB15MFP_EPWM1_CH0 );		
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC0MFP_UART2_RXD | SYS_GPC_MFPL_PC1MFP_UART2_TXD);

    PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE5_Msk);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk |
                       SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_EADC0_CH0 | SYS_GPB_MFPL_PB1MFP_EADC0_CH1 |
                      SYS_GPB_MFPL_PB2MFP_EADC0_CH2 | SYS_GPB_MFPL_PB5MFP_EADC0_CH5);
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT5|BIT2|BIT1|BIT0);

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    SYS_Init();
	
    UART0_Init();
    UART2_Init();
	
	GPIO_Init();
	
	TIMER0_Init();

	/*
		UART0 : PB12 / PB13 , 9600 N 8 1 , communication
		UART2 : PC0 / PC1 , 115200 N 8 1 , debug

		EPWM0_CH4 : PA1 , 30K , resolution 12 bit
		EPWM1_CH0 : PB15 , CAP

		EADC_CH5 : PB5 , trigger by PWM

		EADC_CH8~10 : PB0~2
	
	*/


    EPWM_Init();
	
	EPWM_CAP_PDMA_Init();
	EPWM_CAP_Init();

	ADC_Convert_Init();
	
    /* Got no where to go, just loop forever */
    while(1)
    {
		LED_Y ^= 1;

		EADC_Convert_Result();
		CaptureCalculation();
    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
