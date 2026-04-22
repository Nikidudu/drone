// imu_madg_filter.h
#ifndef IMU_MADG_FILTER_H
#define IMU_MADG_FILTER_H

void madgwick_init(void);

void madgwick_update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt);

void madgwick_get_euler(float *roll, float *pitch, float *yaw);

#endif