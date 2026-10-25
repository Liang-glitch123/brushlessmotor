#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f4xx_hal.h"

#define ADC_REGULAR_CHANNEL_COUNT       2U
#define ADC_REGULAR_FRAME_COUNT         32U
#define ADC_NUM_MAX                     (ADC_REGULAR_CHANNEL_COUNT * ADC_REGULAR_FRAME_COUNT)
#define ADC_INJECTED_CHANNEL_COUNT      3U

#define VREF                            3.3f
#define EMF_OUTPUT_OFFSET               1.24f
#define EMF_ISOLATION_GAIN              1.0f

/* Current-sense calibration for the fitted amplifier and shunt. */
#define GET_ADC_CURR(val)               ((((float)(val) * 3.3f / 4096.0f) - 1.24f) / 8.0f / 0.02f * 1000.0f)
#define ADC_RAW_TO_VOLT(val)            ((float)(val) * VREF / 4096.0f)
#define GET_VBUS_VAL(val)               ((ADC_RAW_TO_VOLT(val) - 1.24f) * 37.0f)
#define GET_EMF_VAL(val)                ((ADC_RAW_TO_VOLT(val) - EMF_OUTPUT_OFFSET) / EMF_ISOLATION_GAIN)

extern ADC_HandleTypeDef hadc1;
extern uint8_t flag;
extern volatile int32_t current_u;
extern volatile int32_t current_v;
extern volatile int32_t current_w;
extern volatile uint32_t tim1_update_count;
extern volatile uint32_t tim1_cc4_event_count;
extern volatile uint32_t adc1_injected_complete_count;
extern volatile uint32_t adc2_injected_complete_count;
extern volatile uint16_t adc_current_raw[ADC_INJECTED_CHANNEL_COUNT];
extern volatile uint16_t adc_phase_voltage_raw[ADC_INJECTED_CHANNEL_COUNT];

void ADC_Init(void);
void capture_injected_samples(void);
void adc_process(void);
int32_t get_curr_val_u(void);
int32_t get_curr_val_v(void);
int32_t get_curr_val_w(void);
uint16_t get_ibus_adc_raw(void);
float get_vbus_val(void);
float get_emf_u_val(void);
float get_emf_v_val(void);
float get_emf_w_val(void);

#endif
