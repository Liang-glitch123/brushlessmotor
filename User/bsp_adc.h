#ifndef __BSP_ADC_H
#define	__BSP_ADC_H

#include "stm32f4xx.h"

#define VREF                            3.3f     // 参考电压，理论上是3.3
#define ADC_NUM_MAX                     320    // ADC 转换结果缓冲区最大值

#define GET_ADC_VDC_VAL(val)            ((float)val/4096.0f*VREF)          // 得到电压值

#define TEMP_ADC_GPIO_PORT              GPIOF
#define TEMP_ADC_GPIO_PIN               GPIO_PIN_10

#define TEMP_ADC_CHANNEL                ADC_CHANNEL_8

#define CURR_U_ADC_GPIO_PORT              GPIOA
#define CURR_U_ADC_GPIO_PIN               GPIO_PIN_3

#define CURR_U_ADC_CHANNEL                ADC_CHANNEL_3

#define CURR_V_ADC_GPIO_PORT              GPIOA
#define CURR_V_ADC_GPIO_PIN               GPIO_PIN_4

#define CURR_V_ADC_CHANNEL                ADC_CHANNEL_4

#define CURR_W_ADC_GPIO_PORT              GPIOA
#define CURR_W_ADC_GPIO_PIN               GPIO_PIN_6

#define CURR_W_ADC_CHANNEL                ADC_CHANNEL_6

#define GET_ADC_CURR_VAL(val)           (((float)val)/(float)8.0/(float)0.02*(float)1000.0)        // 得到电流值，电压放大8倍，0.02是采样电阻，单位mA。

#define VBUS_GPIO_PORT                  GPIOB
#define VBUS_GPIO_PIN                   GPIO_PIN_0

#define VBUS_ADC_CHANNEL                ADC_CHANNEL_8

#define GET_VBUS_VAL(val)               (((float)val - 1.24f) * 37.0f )      // 获取电压值（测量电压是电源电压的1/37）

extern ADC_HandleTypeDef ADC_Handle;

int32_t get_curr_val_v(void);//获取V相的电流值
int32_t get_curr_val_u(void);//获取U相的电流值
int32_t get_curr_val_w(void);//获取W相的电流值
void ADC_Init(void);				//ADC的初始化
float get_ntc_v_val(void);	//获取温度传感器端的电压值	
float get_ntc_r_val(void);	//获取温度传感器端的电阻值
float get_ntc_t_val(void);	//获取温度传感器的温度
float get_vbus_val(void);		//获取电源电压值
void adc_process(void);
 
#endif /* __BSP_ADC_H */
