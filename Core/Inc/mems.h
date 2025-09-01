#ifndef _MEMS_H_
#define _MEMS_H_

void icm42688_reset_quat_identity(void);

void icm42688_update_quat(icm42688RawData_t raw_g, icm42688RawData_t raw_a, float dt, float q_out[4]);

void icm42688_pipeline_update(const icm42688RawData_t *rawA, const icm42688RawData_t *rawG, float dt, float q_out[4]);

void icm42688_pipeline_reset(void);

#endif
