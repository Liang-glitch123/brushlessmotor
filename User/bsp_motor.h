#ifndef __BSP_MOTOR_TIM_H
#define	__BSP_MOTOR_TIM_H

#include "stm32f4xx.h"


#include "tim.h"
#define htimx_bldcm htim1

#define PWM_PERIOD_COUNT     (5600)
#define PWM_MAX_PERIOD_COUNT    (PWM_PERIOD_COUNT - 100)
#define PWM_PRESCALER_COUNT     (2)

#define htimx_hall htim3

#define HALL_PERIOD_COUNT     (0xFFFF)
#define HALL_PRESCALER_COUNT     (128)

void TIMx_Configuration(void);
void stop_pwm_output(void);
void set_pwm_pulse(uint16_t pulse);

void hall_enable(void);
void hall_disable(void);
void hall_tim_config(void);

#endif /* __BSP_MOTOR_TIM_H */

