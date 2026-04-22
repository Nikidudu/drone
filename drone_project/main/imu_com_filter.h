#ifndef IMU_COM_FILTER_H
#define IMU_COM_FILTER_H

void filter_init(void);
void filter_update(float ax, float ay, float az,
                   float gx, float gy, float gz,
                   float *out_roll_gyro,
                   float *out_roll_accel,
                   float *out_roll_comp,
                   float *out_pitch_gyro,
                   float *out_pitch_accel,
                   float *out_pitch_comp);

#endif