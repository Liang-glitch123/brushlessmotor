#include "bsp_motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
#define htimx_bldcm htim1
#define htimx_hall htim3

static uint16_t bldcm_pulse = 0;

motor_rotate_t motor_drive = {0};    // 定义电机驱动管理结构体

/**
  * @brief  停止pwm输出
  * @param  无
  * @retval 无
  */
void stop_pwm_output(void)
{
  /* 关闭定时器通道1输出PWM */
  __HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);

  /* 关闭定时器通道2输出PWM */
  __HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);
  
  /* 关闭定时器通道3输出PWM */
  __HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);    // 关闭下桥臂
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);    // 关闭下桥臂
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);    // 关闭下桥臂
}

/**
  * @brief  设置pwm输出的占空比
  * @param  pulse:要设置的占空比
  * @retval 无
  */
void set_pwm_pulse(uint16_t pulse)
{
  /* 设置定时器通道输出 PWM 的占空比 */
	bldcm_pulse = pulse;
}

/**
  * @brief  使能霍尔传感器
  * @param  无
  * @retval 无
  */
void hall_enable(void)
{
  /* 使能霍尔传感器接口 */
  __HAL_TIM_ENABLE_IT(&htimx_hall, TIM_IT_TRIGGER);
  __HAL_TIM_ENABLE_IT(&htimx_hall, TIM_IT_UPDATE);
  
  HAL_TIMEx_HallSensor_Start_IT(&htimx_hall);
  
  HAL_TIM_TriggerCallback(&htimx_hall);   // 执行一次换相
}

/**
  * @brief  禁用霍尔传感器
  * @param  无
  * @retval 无
  */
void hall_disable(void)
{
  /* 禁用霍尔传感器接口 */
  __HAL_TIM_DISABLE_IT(&htimx_hall, TIM_IT_TRIGGER);
  __HAL_TIM_DISABLE_IT(&htimx_hall, TIM_IT_UPDATE);
  HAL_TIMEx_HallSensor_Stop(&htimx_hall);
}

uint8_t get_hall_state(void)
{
  uint8_t state = 0;
  
#if 1
  /* 读取霍尔传感器 U 的状态 */
  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 0;
  }
  
  /* 读取霍尔传感器 V 的状态 */
  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 1;
  }
  
  /* 读取霍尔传感器 W 的状态 */
  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) != GPIO_PIN_RESET)
  {
    state |= 0x01U << 2;
  }
#else
  state = (GPIOH->IDR >> 10) & 7;    // 读 3 个霍尔传感器的状态
#endif

  return state;    // 返回传感器状态
}

int update = 0;     // 定时器更新计数

/**
  * @brief  更新电机实际速度方向与位置
  * @param  dir_in：霍尔值
  * @retval 无
  */
static uint8_t count = 0;
static void update_speed_location_dir(uint8_t dir_in)
{
  uint8_t step[6] = {1, 3, 2, 6, 4, 5};

  static uint8_t num_old = 0;
  uint8_t step_loc = 0;    // 记录当前霍尔位置
  int8_t dir = 1;
  
  for (step_loc=0; step_loc<6; step_loc++)
  {
    if (step[step_loc] == dir_in)    // 找到当前霍尔的位置
    {
      break;
    }
  }
  
  /* 端点处理 */
  if (step_loc == 0)
  {
    if (num_old == 1)
    {
      dir = 1;
    }
    else if (num_old == 5)
    {
      dir = -1;
    }
  }
  /* 端点处理 */
  else if (step_loc == 5)
  {
    if (num_old == 0)
    {
      dir = 1;
    }
    else if (num_old == 4)
    {
      dir = -1;
    }
  }
  else if (step_loc > num_old)
  {
    dir = -1;
  }
  else if (step_loc < num_old)
  {
    dir = 1;
  }
  
  num_old = step_loc;
//  motor_drive.speed *= dir;;
	motor_drive.speed_group[count-1]*= dir;
  motor_drive.location += dir;    // 更新位置
//	printf("位置：%d\r\n", motor_drive.location);
}

/**
  * @brief  更新电机速度
  * @param  time：计数器的总值
  * @param  num：霍尔触发次数
  * @retval 无
  */
static void update_motor_speed(uint8_t dir_in, uint32_t time)
{
  int speed_temp = 0;
  static int flag = 0;
  float f = 0;

  /* 计算速度：
     电机每转一圈共用12个脉冲，(1.0/(84000000.0/128.0)为计数器的周期，(1.0/(84000000.0/128.0) * time)为时间长。
  */

  if (time == 0)
    motor_drive.speed_group[count++] = 0;
  else
  {
    f = (1.0f / (84000000.0f / HALL_PRESCALER_COUNT) * time);
    f = (1.0f / 12.0f) / (f  / 60.0f);
    motor_drive.speed_group[count++] = f;
  }
	update_speed_location_dir(dir_in);
//	motor_drive.speed = motor_drive.speed_group[count-1];
  if (count >= SPEED_FILTER_NUM)
  {
    flag = 1;
    count = 0;
  }
//	return ;
  speed_temp = 0;
	
  /* 计算近 SPEED_FILTER_NUM 次的速度平均值（滤波） */
  if (flag)
  {
    for (uint8_t c=0; c<SPEED_FILTER_NUM; c++)
    {
      speed_temp += motor_drive.speed_group[c];
    }

    motor_drive.speed = speed_temp/ SPEED_FILTER_NUM;
  }
  else
  {
    for (uint8_t c=0; c<count; c++)
    {
      speed_temp += motor_drive.speed_group[c];
    }

    motor_drive.speed = speed_temp / count;
  }
}

/**
  * @brief  霍尔传感器触发回调函数
  * @param  htim:定时器句柄
  * @retval 无
  */
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{
  /* 获取霍尔传感器引脚状态,作为换相的依据 */
  uint8_t step = 0;
  step = get_hall_state();

	if(get_bldcm_direction() == MOTOR_REV)
	{
		step = 7 - step;
	}
  if (htim == &htimx_hall)   // 判断是否由触发中断产生
  {
    update_motor_speed(step, __HAL_TIM_GET_COMPARE(htim,TIM_CHANNEL_1));
    motor_drive.timeout = 0;
  }
	switch(step)
	{
		case 1:    /* U+ W- */
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // 通道 2 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);    // 关闭下桥臂
		
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // 通道 3 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);    // 关闭下桥臂

			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, bldcm_pulse);                  // 通道 1 配置的占空比
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);      // 开启下桥臂
			break;
		
		case 2:     /* V+ U- */
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // 通道 3 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);    // 关闭下桥臂

			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // 通道 1 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);    // 关闭下桥臂
		
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, bldcm_pulse);                  // 通道 2 配置的占空比
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);      // 开启下桥臂
		
			break;
		
		case 3:    /* V+ W- */
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // 通道 1 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);    // 关闭下桥臂

			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // 通道 3 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);    // 关闭下桥臂
			
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, bldcm_pulse);                  // 通道 2 配置的占空比
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);      // 开启下桥臂
			break;
		
		case 4:     /* W+ V- */
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // 通道 1 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);    // 关闭下桥臂

			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // 通道 2 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);    // 关闭下桥臂
 
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, bldcm_pulse);                  // 通道 3 配置的占空比
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);      // 开启下桥臂 
			break;
		
		case 5:     /* U+  V -*/
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, 0);                            // 通道 3 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);    // 关闭下桥臂
		
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // 通道 2 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);    // 关闭下桥臂
		
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, bldcm_pulse);                  // 通道 1 配置的占空比
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);      // 开启下桥臂
			break;
		
		case 6:     /* W+ U- */
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_2, 0);                            // 通道 2 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);    // 关闭下桥臂
		
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_1, 0);                            // 通道 1 配置为 0
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);    // 关闭下桥臂
		
			__HAL_TIM_SET_COMPARE(&htimx_bldcm, TIM_CHANNEL_3, bldcm_pulse);                  // 通道 3 配置的占空比
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);      // 开启下桥臂
			break;
	}
  
  HAL_TIM_GenerateEvent(&htimx_bldcm, TIM_EVENTSOURCE_COM);    // 软件产生换相事件，此时才将配置写入

  update = 0;
}

/**
  * @brief  定时器更新中断回调函数
  * @param  htim:定时器句柄
  * @retval 无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (update++ > 1)    // 有一次在产生更新中断前霍尔传感器没有捕获到值
  {
    update = 0;
    
    /* 堵转超时停止 PWM 输出 */
    hall_disable();       // 禁用霍尔传感器接口
    stop_pwm_output();    // 停止 PWM 输出
  }
}

/*********************************************END OF FILE**********************/
