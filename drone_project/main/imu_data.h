// imu.h
#ifndef IMU_H
#define IMU_H

// Initialise IMU (I2C + sensor setup)
void imu_init(void);

// Calibrate gyro by averaging 500 samples at startup (assumes stationary)
void imu_calibrate_gyro(void);

// Read IMU data (returns values in physical units)
void imu_read(float *ax, float *ay, float *az,
              float *gx, float *gy, float *gz);

#endif