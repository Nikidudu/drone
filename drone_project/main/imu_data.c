#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu_data.h"

#define TAG "MPU6500"

// I2C config
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

#define MPU6500_ADDR 0x68

// Registers
#define PWR_MGMT_1     0x6B
#define ACCEL_XOUT_H   0x3B

// settings
#define ACCEL_SENS 16384.0   // ±2g
#define GYRO_SENS 131.0      // ±250 deg/s

// ---------------- I2C INIT ----------------
void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ---------------- WRITE REGISTER ----------------
esp_err_t mpu6500_write(uint8_t reg, uint8_t data)
{
    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        MPU6500_ADDR,
        (uint8_t[]){reg, data},
        2,
        pdMS_TO_TICKS(100)
    );
}

// ---------------- READ REGISTER ----------------
esp_err_t mpu6500_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,
        MPU6500_ADDR,
        &reg,
        1,
        data,
        len,
        pdMS_TO_TICKS(100)
    );
}

// ---------------- INIT MPU ----------------
void mpu6500_init()
{
    // Wake up device (clear sleep bit)
    mpu6500_write(PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
}

// ---------------- READ DATA ----------------
void mpu6500_read_data()
{
    uint8_t raw_data[14];

    // Read accel + temp + gyro (14 bytes total)
    mpu6500_read(ACCEL_XOUT_H, raw_data, 14);

    int16_t ax = (raw_data[0] << 8) | raw_data[1];
    int16_t ay = (raw_data[2] << 8) | raw_data[3];
    int16_t az = (raw_data[4] << 8) | raw_data[5];

    int16_t gx = (raw_data[8] << 8) | raw_data[9];
    int16_t gy = (raw_data[10] << 8) | raw_data[11];
    int16_t gz = (raw_data[12] << 8) | raw_data[13];

    // Convert to physical units
    float accel_x = ax / ACCEL_SENS;
    float accel_y = ay / ACCEL_SENS;
    float accel_z = az / ACCEL_SENS;

    float gyro_x = gx / GYRO_SENS;
    float gyro_y = gy / GYRO_SENS;
    float gyro_z = gz / GYRO_SENS;

    // ESP_LOGI(TAG, "ACCEL [g]: X=%.2f Y=%.2f Z=%.2f", accel_x, accel_y, accel_z);
    // ESP_LOGI(TAG, "GYRO [deg/s]: X=%.2f Y=%.2f Z=%.2f", gyro_x, gyro_y, gyro_z);

    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
       accel_x, accel_y, accel_z,
       gyro_x, gyro_y, gyro_z);
}