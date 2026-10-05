#include "bsp_key.h"
#include "bsp_focm_control.h"

uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/*����Ƿ��а������� */
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON )  
	{	 
		/*�ȴ������ͷ� */
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON);   
		return 	KEY_ON;	 
	}
	else
		return KEY_OFF;
}

static float focm_speed = 10.0f;
uint8_t i = 0;
void Key_process(void)
{
	/* ɨ��KEY1 */
	if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
	{
		set_focm_current(0.0f, 1.0f);
        set_focm_speed(focm_speed);
		set_focm_enable();
	}
	
	/* ɨ��KEY2 */
	if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
	{
		set_focm_disable();
	}
	
	/* ɨ��KEY3 */
	if( Key_Scan(KEY3_GPIO_PORT, KEY3_PIN) == KEY_ON)
	{
		focm_speed += 1.0f;
        if (focm_speed > 30.0f) focm_speed = 30.0f;
		
		set_focm_current(0.0f, 1.0f);
        set_focm_speed(focm_speed);
	}
	
	/* ɨ��KEY4 */
	if( Key_Scan(KEY4_GPIO_PORT, KEY4_PIN) == KEY_ON)
	{
		focm_speed -= 1.0f;
        if (focm_speed < 0.0f) focm_speed = 0.0f;

		set_focm_current(0.0f, 1.0f);
        set_focm_speed(focm_speed);
	}
	
	/* ɨ��KEY5 */
	if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
	{
		focm_reverse();
	}
}
