#include "bsp_adc.h"
#include "bsp_focm_control.h"
#include <math.h>

extern ADC_HandleTypeDef hadc1;


static int16_t adc_buff[ADC_NUM_MAX];    // 电压采集缓冲区
int16_t vbus_adc_mean = 0;        // 电源电压 ACD 采样结果平均值
int16_t emf_u_adc_mean = 0;
int16_t emf_v_adc_mean = 0;
int16_t emf_w_adc_mean = 0;
uint32_t adc_mean_t = 0;        // 平均值累加
uint32_t adc_mean_sum_u = 0;        // 平均值累加
uint32_t adc_mean_sum_v = 0;        // 平均值累加
uint32_t adc_mean_sum_w = 0;        // 平均值累加
static uint32_t adc_mean_count_u = 0;      // 累加计数
static uint32_t adc_mean_count_v = 0;      // 累加计数
static uint32_t adc_mean_count_w = 0;      // 累加计数

/**
  * @brief  常规转换在非阻塞模式下完成回调
  * @param  hadc: ADC  句柄.
  * @retval 无
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	int32_t adc_mean = 0;
  /* DMA循环模式持续采样。 */
  
  /* 计算温度通道采样的平均值 */
  for(uint32_t count = 4; count < ADC_NUM_MAX; count+=8)
  {
    adc_mean += (int32_t)adc_buff[count];
  }
	  adc_mean_t = adc_mean / (ADC_NUM_MAX / 8);    // 保存平均值
		adc_mean = 0;

  
  /* 计算电压通道采样的平均值 */
  for(uint32_t count = 3; count < ADC_NUM_MAX; count+=8)
  {
    adc_mean += (int32_t)adc_buff[count];
  }
  
  vbus_adc_mean = adc_mean / (ADC_NUM_MAX / 8);    // 保存平均值
  adc_mean = 0;

  /* ADC_IN10/IN12/IN13：Motor1_EMFU/EMFV/EMFW */
  for(uint32_t count = 5; count < ADC_NUM_MAX; count += 8)
    adc_mean += (int32_t)adc_buff[count];
  emf_u_adc_mean = adc_mean / (ADC_NUM_MAX / 8);
  adc_mean = 0;
  for(uint32_t count = 6; count < ADC_NUM_MAX; count += 8)
    adc_mean += (int32_t)adc_buff[count];
  emf_v_adc_mean = adc_mean / (ADC_NUM_MAX / 8);
  adc_mean = 0;
  for(uint32_t count = 7; count < ADC_NUM_MAX; count += 8)
    adc_mean += (int32_t)adc_buff[count];
  emf_w_adc_mean = adc_mean / (ADC_NUM_MAX / 8);
  adc_mean = 0;
#if 1 
		  /* 计算电流通道采样的平均值 */
   for(uint32_t count = 0; count < ADC_NUM_MAX; count+=8)
  {
    adc_mean += (uint32_t)adc_buff[count];
  }
  
  adc_mean_sum_u += adc_mean / (ADC_NUM_MAX / 8);    // 累加电压
  adc_mean_count_u++;
	  adc_mean = 0;
			  /* 计算电流通道采样的平均值 */
   for(uint32_t count = 1; count < ADC_NUM_MAX; count+=8)
  {
    adc_mean += (uint32_t)adc_buff[count];
  }
  
  adc_mean_sum_v += adc_mean / (ADC_NUM_MAX / 8);    // 累加电压
  adc_mean_count_v++;
		adc_mean = 0;
			  /* 计算电流通道采样的平均值 */
   for(uint32_t count = 2; count < ADC_NUM_MAX; count+=8)
  {
    adc_mean += (uint32_t)adc_buff[count];
  }
  
  adc_mean_sum_w += adc_mean / (ADC_NUM_MAX / 8);    // 累加电压
  adc_mean_count_w++;	adc_mean = 0;
#else
	  vbus_adc_mean = adc_buff[1];
	      /* 计算电流通道采样的平均值 */
  
#endif


  

}

void ADC_Init(void)
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buff, ADC_NUM_MAX);
}

/**
  * @brief  获取温度传感器端的电压值
  * @param  无
  * @retval 转换得到的电压值
  */
float get_ntc_v_val(void)
{
  float vdc = GET_ADC_VDC_VAL(adc_mean_t);      // 获取电压值
  
  return vdc;
}

/**
  * @brief  获取温度传感器端的电阻值
  * @param  无
  * @retval 转换得到的电阻值
  */
float get_ntc_r_val(void)
{
  float r = 0;
  float vdc = get_ntc_v_val();
  
  r = (VREF - vdc) / (vdc / (float)4700.0);
  
  return r;
}

/**
  * @brief  获取温度传感器的温度
  * @param  无
  * @retval 转换得到的温度，单位：（℃）
  */
float get_ntc_t_val(void)
{
  float t = 0;             // 测量温度
  float Rt = 0;            // 测量电阻
  float Ka = 273.15;       // 0℃ 时对应的温度（开尔文）
  float R25 = 10000.0;     // 25℃ 电阻值
  float T25 = Ka + 25;     // 25℃ 时对应的温度（开尔文）
  float B = 3950.0;        /* B-常数：B = ln(R25 / Rt) / (1 / T – 1 / T25)，
                             其中 T = 25 + 273.15 */

  Rt = get_ntc_r_val();    // 获取当前电阻值

  t = B * T25 / (B + log(Rt / R25) * T25) - Ka ;    // 使用公式计算

  return t;
}
/**
  * @brief  获取V相的电流值
  * @param  无
  * @retval 转换得到的电流值
  */
int32_t get_curr_val_v(void)
{
  static uint8_t flag = 0;
	static uint32_t adc_offset = 0;    // 偏置电压
  int16_t curr_adc_mean = 0;         // 电流 ACD 采样结果平均值
  
  if (adc_mean_count_v == 0) return 0;
  curr_adc_mean = adc_mean_sum_v / adc_mean_count_v;    // 保存平均值
  

    adc_mean_count_v = 0;
    adc_mean_sum_v = 0;
    
    if (flag < 17)
    {
      adc_offset = curr_adc_mean;    // 多次记录偏置电压，待系统稳定偏置电压才为有效值
      flag += 1;
    }
    if(curr_adc_mean>=adc_offset)
	{
		curr_adc_mean -= adc_offset;                     // 减去偏置电压
	}else
	{
		curr_adc_mean=0;
	}

  float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // 获取电压值
  
  return GET_ADC_CURR_DIFF(vdc);
}
/**
  * @brief  获取U相的电流值
  * @param  无
  * @retval 转换得到的电流值
  */
int32_t get_curr_val_u(void)
{
  static uint8_t flag = 0;
  static uint32_t adc_offset = 0;    // 偏置电压
  int16_t curr_adc_mean = 0;         // 电流 ACD 采样结果平均值
  
  if (adc_mean_count_u == 0) return 0;
  curr_adc_mean = adc_mean_sum_u / adc_mean_count_u;    // 保存平均值
  

    adc_mean_count_u = 0;
    adc_mean_sum_u = 0;
    
    if (flag < 17)
    {
      adc_offset = curr_adc_mean;    // 多次记录偏置电压，待系统稳定偏置电压才为有效值
      flag += 1;
    }
    if(curr_adc_mean>=adc_offset)
	{
		curr_adc_mean -= adc_offset;                     // 减去偏置电压
	}else
	{
		curr_adc_mean=0;
	}

  float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // 获取电压值
  
  return GET_ADC_CURR_DIFF(vdc);
}
/**
  * @brief  获取W相的电流值
  * @param  无
  * @retval 转换得到的电流值
  */
int32_t get_curr_val_w(void)
{
  static uint8_t flag = 0;
  static uint32_t adc_offset = 0;    // 偏置电压
  int16_t curr_adc_mean = 0;         // 电流 ACD 采样结果平均值
  
  if (adc_mean_count_w == 0) return 0;
  curr_adc_mean = adc_mean_sum_w / adc_mean_count_w;    // 保存平均值
  

    adc_mean_count_w = 0;
    adc_mean_sum_w = 0;
    
    if (flag < 17)
    {
      adc_offset = curr_adc_mean;    // 多次记录偏置电压，待系统稳定偏置电压才为有效值
      flag += 1;
    }
    if(curr_adc_mean>=adc_offset)
	{
		curr_adc_mean -= adc_offset;                     // 减去偏置电压
	}else
	{
		curr_adc_mean=0;
	}

  float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // 获取电压值
  
  return GET_ADC_CURR_DIFF(vdc);
}
/**
  * @brief  获取电源电压值
  * @param  无
  * @retval 转换得到的电压值
  */
float get_vbus_val(void)
{
  float vdc = GET_ADC_VDC_VAL(vbus_adc_mean);      // 获取电压值
  return GET_VBUS_VAL(vdc);
}

float get_emf_u_val(void)
{
  return GET_ADC_VDC_VAL(emf_u_adc_mean);
}

float get_emf_v_val(void)
{
  return GET_ADC_VDC_VAL(emf_v_adc_mean);
}

float get_emf_w_val(void)
{
  return GET_ADC_VDC_VAL(emf_w_adc_mean);
}

uint8_t flag = 0;
int32_t current_v = 0;
int32_t current_u = 0;
int32_t current_w = 0;
void adc_process(void)
{
	/* 三相电流在同一组DMA数据中同步更新。 */
  if (adc_mean_count_u != 0 && adc_mean_count_v != 0 && adc_mean_count_w != 0)
  {
    current_u = (int32_t)((((float)(adc_mean_sum_u / adc_mean_count_u) - 1540.0f) * VREF / 4096.0f) / CURRENT_SENSE_GAIN / CURRENT_SHUNT_RESISTANCE * 1000.0f);
    current_v = (int32_t)((((float)(adc_mean_sum_v / adc_mean_count_v) - 1540.0f) * VREF / 4096.0f) / CURRENT_SENSE_GAIN / CURRENT_SHUNT_RESISTANCE * 1000.0f);
    current_w = (int32_t)((((float)(adc_mean_sum_w / adc_mean_count_w) - 1540.0f) * VREF / 4096.0f) / CURRENT_SENSE_GAIN / CURRENT_SHUNT_RESISTANCE * 1000.0f);
    adc_mean_sum_u = 0; adc_mean_sum_v = 0; adc_mean_sum_w = 0;
    adc_mean_count_u = 0; adc_mean_count_v = 0; adc_mean_count_w = 0;
  }  
  /* 电流值已在ADC DMA回调中同步更新。 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    focm_control_step((float)current_u * 0.001f,
                      (float)current_v * 0.001f,
                      (float)current_w * 0.001f,
                      get_vbus_val());
  }
}
