#ifndef __ICM42688_HARD_I2C_H_
#define __ICM42688_HARD_I2C_H_

#include <stdbool.h>

bool icm42688_init(void);

void init_42688(void);

void iic_get_data(icm42688RawData_t *accData, icm42688RawData_t *gyroData);

bool icm42688_read_accel(icm42688RawData_t *acc);

bool icm42688_read_gyro(icm42688RawData_t *gyro);

#endif
