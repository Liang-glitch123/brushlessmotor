#include "bsp_key.h"
#include "bsp_focm_control.h"

uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/* 检查按键是否按下。 */
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON )  
	{	 
		/* 等待按键释放。 */
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON);   
		return 	KEY_ON;	 
	}
	else
		return KEY_OFF;
}

float omega_ref = 300;              // 电角速度 rad/s
void Key_process(void)
{
	/* 扫描 KEY1：启动电机。 */
	if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
	{
		set_focm_current(0.0f, 1.0f);
		set_focm_speed(omega_ref);
		set_focm_enable();
	}
	
	/* 扫描 KEY2：停止电机。 */
	if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
	{
		set_focm_disable();
	}
	
	/* 扫描 KEY3：加速。 */
	if( Key_Scan(KEY3_GPIO_PORT, KEY3_PIN) == KEY_ON)
	{
		omega_ref += 50.0f;
    if (omega_ref > 2000.0f) omega_ref = 2000.0f;
		set_focm_speed(omega_ref);
		set_focm_current(0.0f, 1.0f);
	}
	
	/* 扫描 KEY4：减速。 */
	if( Key_Scan(KEY4_GPIO_PORT, KEY4_PIN) == KEY_ON)
	{
		omega_ref -= 50.0f;
    if (omega_ref < 300.0f) omega_ref = 300.0f;
		set_focm_speed(omega_ref);
		set_focm_current(0.0f, 1.0f);
	}
	
	/* 扫描 KEY5：切换旋转方向。 */
	if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
	{
		omega_ref = 300.0f;
		set_focm_speed(omega_ref);
		focm_reverse();
	}
}
