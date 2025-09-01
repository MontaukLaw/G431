#ifndef _BAT_VAL_H_
#define _BAT_VAL_H_

uint8_t bat_mini_volt_to_percentage(uint16_t mvolts);

void bat_task(void);

extern uint16_t bat_val_dma_buf[];

#endif
