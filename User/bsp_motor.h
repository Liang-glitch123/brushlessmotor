#ifndef __BSP_MOTOR_TIM_H
#define	__BSP_MOTOR_TIM_H

#include "stm32f4xx.h"
#include "bsp_bldcm_control.h"

#include "tim.h"
#define htimx_bldcm htim1

#define SPEED_FILTER_NUM      25    // 速度滤波次数

typedef struct
{
  int32_t timeout;            // 定时器更新计数
  float speed;                // 电机速度 rps（转/分钟）
  int32_t enable_flag;        // 电机使能标志
  int32_t speed_group[SPEED_FILTER_NUM];
  int32_t location;
}motor_rotate_t;

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

