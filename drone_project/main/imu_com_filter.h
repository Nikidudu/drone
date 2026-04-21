// imu__com_filter.h
#ifndef IMU_COM_FILTER_H
#define IMU_COM_FILTER_H

void filter_init(void);



// Complementary filter update function
void filter_update(float ax, float ay, float az,
                   float gx, float gy, float gz,
                   float *roll, float *pitch);



#endif