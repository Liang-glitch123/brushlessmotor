#ifndef __BSP_FOCM_CONTROL_H
#define __BSP_FOCM_CONTROL_H
#include "stm32f4xx_hal.h"

#define PWM_PERIOD_COUNT (5600U)
#define PWM_MAX_PERIOD_COUNT (PWM_PERIOD_COUNT - 100U)

typedef struct { 
	float ia,ib,ic;
	float id,iq;
	float id_ref,iq_ref;
	float theta;
	float udc;
	float duty_u,duty_v,duty_w; 
	uint8_t enabled; 
} focm_data_t;

/** 初始化FOC控制模块。 */
void focm_init(void); 
/** 设置d/q轴电流给定。 */
void set_focm_current(float id_ref,float iq_ref); 
void set_focm_angle(float angle);
/* 设置机械角速度，单位 rad/s；模块内部按极对数换算为电角速度。 */
/** 设置机械角速度。 */
void set_focm_speed(float mechanical_speed);
/** 切换旋转方向。 */
void focm_reverse(void);
/** 使能电机PWM输出。 */
void set_focm_enable(void); 
/** 禁止电机PWM输出。 */
void set_focm_disable(void); 
/** 执行一次FOC控制计算。 */
void focm_control_step(float ia,float ib,float ic,float udc); 
/** 获取FOC运行状态。 */
const focm_data_t *get_focm_data(void);

#endif
