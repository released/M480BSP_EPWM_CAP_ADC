# M480BSP_EPWM_CAP_ADC
 M480BSP_EPWM_CAP_ADC

update @ 2020/09/11

1. use EPWM1_CH0 (PB15) with PDMA , to capture signal freq input : EPWM0_CH4 (PA1) , with 30K , different duty

2. use EPWM0_CH4 (PA1) to trigger EADC_CH5 (PB5) , center align , EPWM_TRG_ADC_EVEN_PERIOD

upper channel : EPWM0_CH4 (PA1) , lower channel : use GPIO toggle in EADC00_IRQHandler to indicate ADC converted)

EPWM_TRG_ADC_EVEN_PERIOD (will see GPIO toggle in middle of duty high level)

![image](https://github.com/released/M480BSP_EPWM_CAP_ADC/blob/EVEN_PERIOD_50.jpg)

![image](https://github.com/released/M480BSP_EPWM_CAP_ADC/blob/EVEN_PERIOD_85.jpg)

3. EADC_CH5 (PB5) trigger by EPWM ,  will get result under EADC00_IRQHandler

4. EADC_CH0 (PB0),EADC_CH1 (PB1),EADC_CH2 (PB2),  will get result under EADC01_IRQHandler

5. also enable PDMA to get ADC result , under ENABLE_ADC_LOG will display 2 different method to get ADC value

6. enable ENABLE_CAP_LOG , will display capture freq/duty to UART terminal

![image](https://github.com/released/M480BSP_EPWM_CAP_ADC/blob/master/LOG_CAP_50.jpg)

![image](https://github.com/released/M480BSP_EPWM_CAP_ADC/blob/master/LOG_CAP_85.jpg)

7. enable ENABLE_ADC_LOG , will display ADC result to UART terminal

![image](https://github.com/released/M480BSP_EPWM_CAP_ADC/blob/master/LOG_ADC.jpg)

8. below use different PWM counter setting to trigger ADC  

upper channel : EPWM0_CH4 (PA1) , lower channel : use GPIO toggle in EADC00_IRQHandler to indicate ADC converted)

EPWM_TRG_ADC_EVEN_ZERO (will see GPIO toggle in middle of duty low level)

![image](https://github.com/released/M480BSP_EPWM_CAP_ADC/blob/EVEN_ZERO_50.jpg)

EPWM_TRG_ADC_EVEN_ZERO_PERIOD (will see GPIO toggle in middle of duty high and duty low level)

![image](https://github.com/released/M480BSP_EPWM_CAP_ADC/blob/EVEN_ZERO_PERIOD_50.jpg)

EPWM_TRG_ADC_EVEN_COMPARE_UP (will see GPIO toggle in duty high level)

![image](https://github.com/released/M480BSP_EPWM_CAP_ADC/blob/EVEN_COMPARE_UP_50.jpg)

EPWM_TRG_ADC_EVEN_COMPARE_DOWN (will see GPIO toggle in duty low level)

![image](https://github.com/released/M480BSP_EPWM_CAP_ADC/blob/EVEN_COMPARE_DOWN_50.jpg)


