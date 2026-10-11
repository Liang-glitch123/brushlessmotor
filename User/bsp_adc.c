#include "bsp_adc.h"
#include "adc.h"
#include "bsp_focm_control.h"

#define ADC_INJECTED_CHANNEL_COUNT      3U

static uint16_t adc_buff[ADC_NUM_MAX];
static volatile uint16_t ibus_adc_raw;
static volatile uint16_t vbus_adc_mean;
static volatile int32_t phase_current_ma[2][ADC_INJECTED_CHANNEL_COUNT];
static volatile uint16_t phase_emf_adc[2][ADC_INJECTED_CHANNEL_COUNT];
static volatile uint8_t phase_current_buffer;
static volatile uint8_t phase_current_ready;

uint8_t flag = 0U;
volatile int32_t current_u = 0;
volatile int32_t current_v = 0;
volatile int32_t current_w = 0;

static void read_phase_currents(int32_t *current_u_ma,
                                int32_t *current_v_ma,
                                int32_t *current_w_ma)
{
  uint8_t buffer = phase_current_buffer;
  *current_u_ma = phase_current_ma[buffer][0];
  *current_v_ma = phase_current_ma[buffer][1];
  *current_w_ma = phase_current_ma[buffer][2];
}

static void read_phase_emf(uint16_t *emf_u_raw,
                           uint16_t *emf_v_raw,
                           uint16_t *emf_w_raw)
{
  uint8_t buffer = phase_current_buffer;
  *emf_u_raw = phase_emf_adc[buffer][0];
  *emf_v_raw = phase_emf_adc[buffer][1];
  *emf_w_raw = phase_emf_adc[buffer][2];
}

static void process_regular_block(const uint16_t *block, uint32_t sample_count)
{
  uint32_t sum[ADC_REGULAR_CHANNEL_COUNT] = {0U};
  uint32_t frame_count = sample_count / ADC_REGULAR_CHANNEL_COUNT;
  uint32_t frame;

  for (frame = 0U; frame < frame_count; frame++)
  {
    uint32_t base = frame * ADC_REGULAR_CHANNEL_COUNT;
    uint32_t rank;
    for (rank = 0U; rank < ADC_REGULAR_CHANNEL_COUNT; rank++)
    {
      sum[rank] += block[base + rank];
    }
  }

  vbus_adc_mean = (uint16_t)(sum[0] / frame_count);
  ibus_adc_raw = (uint16_t)(sum[1] / frame_count);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    process_regular_block(adc_buff, ADC_NUM_MAX / 2U);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    process_regular_block(&adc_buff[ADC_NUM_MAX / 2U], ADC_NUM_MAX / 2U);
  }
}

static void capture_injected_samples(void)
{
  uint8_t next_buffer;
  uint32_t raw_u;
  uint32_t raw_v;
  uint32_t raw_w;
  uint32_t raw_emf_u;
  uint32_t raw_emf_v;
  uint32_t raw_emf_w;

  raw_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  raw_v = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  raw_w = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
  raw_emf_u = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  raw_emf_v = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
  raw_emf_w = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);

  next_buffer = (uint8_t)(phase_current_buffer ^ 1U);
  phase_current_ma[next_buffer][0] = (int32_t)GET_ADC_CURR(raw_u);
  phase_current_ma[next_buffer][1] = (int32_t)GET_ADC_CURR(raw_v);
  phase_current_ma[next_buffer][2] = (int32_t)GET_ADC_CURR(raw_w);
  phase_emf_adc[next_buffer][0] = (uint16_t)raw_emf_u;
  phase_emf_adc[next_buffer][1] = (uint16_t)raw_emf_v;
  phase_emf_adc[next_buffer][2] = (uint16_t)raw_emf_w;
  phase_current_buffer = next_buffer;
  phase_current_ready = 1U;
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
  __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC);
}

void ADC_Init(void)
{
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(void *)adc_buff, ADC_NUM_MAX) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

}

int32_t get_curr_val_u(void)
{
  return phase_current_ma[phase_current_buffer][0];
}

int32_t get_curr_val_v(void)
{
  return phase_current_ma[phase_current_buffer][1];
}

int32_t get_curr_val_w(void)
{
  return phase_current_ma[phase_current_buffer][2];
}

uint16_t get_ibus_adc_raw(void)
{
  return ibus_adc_raw;
}

float get_vbus_val(void)
{
  uint16_t raw = vbus_adc_mean;
  return GET_VBUS_VAL(raw);
}

float get_emf_u_val(void)
{
  uint16_t raw;
  uint16_t raw_v;
  uint16_t raw_w;
  read_phase_emf(&raw, &raw_v, &raw_w);
  return GET_EMF_VAL(raw);
}

float get_emf_v_val(void)
{
  uint16_t raw_u;
  uint16_t raw;
  uint16_t raw_w;
  read_phase_emf(&raw_u, &raw, &raw_w);
  return GET_EMF_VAL(raw);
}

float get_emf_w_val(void)
{
  uint16_t raw_u;
  uint16_t raw_v;
  uint16_t raw;
  read_phase_emf(&raw_u, &raw_v, &raw);
  return GET_EMF_VAL(raw);
}

void adc_process(void)
{
  int32_t current_u_ma;
  int32_t current_v_ma;
  int32_t current_w_ma;

  if (phase_current_ready != 0U)
  {
    read_phase_currents(&current_u_ma, &current_v_ma, &current_w_ma);
    current_u = current_u_ma;
    current_v = current_v_ma;
    current_w = current_w_ma;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  int32_t current_u_ma;
  int32_t current_v_ma;
  int32_t current_w_ma;

  if (htim->Instance == TIM1)
  {
    /* CC4 samples both ADC injected groups at mid-period; consume the
       completed current and phase-voltage ranks at the next update. */
//    if ((__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC) != RESET) &&
//        (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOC) != RESET))
//    {
//      capture_injected_samples();
//    }

//    if (phase_current_ready != 0U)
//    {
      read_phase_currents(&current_u_ma, &current_v_ma, &current_w_ma);
		focm_control_step((float)current_u_ma * 0.001f,
											(float)current_v_ma * 0.001f,
											(float)current_w_ma * 0.001f,
											get_vbus_val());
//    }
  }
}
