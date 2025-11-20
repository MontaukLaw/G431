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

void start_adc_collecting(void);

uint16_t ema_u16(uint16_t new_data, uint16_t last_data, uint16_t a_num, uint16_t a_den);

void imu_rest_cmd_task(void);

void delay_ms(uint16_t nms);

void uart_send_100hz(void);

extern uint16_t bat_smooth_mvolts;

extern __IO uint32_t tim7_counter;
extern __IO uint8_t bat_adc_done;
extern volatile uint8_t imu_rest_tx_data[];
extern volatile uint8_t uart_busy;

#endif 
