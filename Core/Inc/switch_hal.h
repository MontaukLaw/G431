#ifndef _SWITCH_HAL_H_
#define _SWITCH_HAL_H_  

void turn_on_input_ch(uint8_t input_ch);

void set_channel_pin(uint8_t ch, GPIO_PinState pin_status);

void set_adc_ch(uint8_t adc_ch);

#endif // _SWITCH_HAL_H_
