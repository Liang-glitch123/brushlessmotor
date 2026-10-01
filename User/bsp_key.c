#include "bsp_key.h"

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

void Key_process(void)
{
	/* …®√ËKEY1 */
	if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
	{

	}
	
	/* …®√ËKEY2 */
	if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
	{

	}
	
	/* …®√ËKEY3 */
	if( Key_Scan(KEY3_GPIO_PORT, KEY3_PIN) == KEY_ON)
	{

	}
	
	/* …®√ËKEY4 */
	if( Key_Scan(KEY4_GPIO_PORT, KEY4_PIN) == KEY_ON)
	{

	}
	
	/* …®√ËKEY5 */
	if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
	{

	}
}
