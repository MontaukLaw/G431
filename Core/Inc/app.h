#ifndef _APP_H_
#define _APP_H_ 

void main_task_adc(void);

void main_task(void);

void uart_send(void);

void init_frame_tail(void);

void send_data(uint8_t frame_id);

void fill_tx_data(void);

void main_task_adc_first(void);

void delay_init(void);

void delay_us(uint32_t nus);

// uint8_t ema_u8(uint8_t new_data, uint8_t last_data, float alpha);
uint8_t ema_u8(uint8_t new_data, uint8_t last_data, uint8_t a_num, uint8_t a_den);

extern __IO uint32_t tim7_counter;
extern __IO uint8_t bat_adc_done;

#endif 
