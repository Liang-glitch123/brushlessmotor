#include "bsp_key.h"
#include "bsp_bldcm_control.h"

uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/*ºÏ≤‚ «∑Ò”–∞¥º¸∞¥œ¬ */
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON )  
	{	 
		/*µ»¥˝∞¥º¸ Õ∑≈ */
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON);   
		return 	KEY_ON;	 
	}
	else
		return KEY_OFF;
}

__IO uint16_t ChannelPulse = PWM_MAX_PERIOD_COUNT/10;
uint8_t i = 0;
void Key_process(void)
{
	/* …®√ËKEY1 */
	if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
	{
		set_bldcm_speed(ChannelPulse);
		set_bldcm_enable();
	}
	
	/* …®√ËKEY2 */
	if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
	{
		set_bldcm_disable();
	}
	
	/* …®√ËKEY3 */
	if( Key_Scan(KEY3_GPIO_PORT, KEY3_PIN) == KEY_ON)
	{
		ChannelPulse += PWM_MAX_PERIOD_COUNT/10;
		
		if(ChannelPulse > PWM_MAX_PERIOD_COUNT)
			ChannelPulse = PWM_MAX_PERIOD_COUNT;
		
		set_bldcm_speed(ChannelPulse);
	}
	
	/* …®√ËKEY4 */
	if( Key_Scan(KEY4_GPIO_PORT, KEY4_PIN) == KEY_ON)
	{
		if(ChannelPulse < PWM_MAX_PERIOD_COUNT/10)
			ChannelPulse = 0;
		else
			ChannelPulse -= PWM_MAX_PERIOD_COUNT/10;

		set_bldcm_speed(ChannelPulse);
	}
	
	/* …®√ËKEY5 */
	if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
	{
		set_bldcm_direction( (++i % 2) ? MOTOR_FWD : MOTOR_REV);
	}
}
