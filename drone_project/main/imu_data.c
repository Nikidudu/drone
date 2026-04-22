#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_data.h"
#include <math.h>

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
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C

// settings
#define ACCEL_SENS 16384.0   // ±2g
#define GYRO_SENS 131.0      // ±250 deg/s

#define GYRO_DEADBAND 0.5f

static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;
static float ax_bias = 0, ay_bias = 0, az_bias = 0;

// ---------------- I2C INIT ----------------
static void i2c_master_init()
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
static esp_err_t mpu6500_write(uint8_t reg, uint8_t data)
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
static esp_err_t mpu6500_read(uint8_t reg, uint8_t *data, size_t len)
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
void imu_init()
{
    i2c_master_init();
    // Wake up device (clear sleep bit)
    mpu6500_write(PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu6500_write(GYRO_CONFIG,  0x00);     // ±250°/s
    mpu6500_write(ACCEL_CONFIG, 0x00);     // ±2g
}

// ---------------- READ DATA ----------------
void imu_read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    uint8_t raw_data[14];

    // Read accel + temp + gyro (14 bytes total)
    mpu6500_read(ACCEL_XOUT_H, raw_data, 14);

    int16_t ax_raw = (raw_data[0] << 8) | raw_data[1];
    int16_t ay_raw = (raw_data[2] << 8) | raw_data[3];
    int16_t az_raw = (raw_data[4] << 8) | raw_data[5];

    int16_t gx_raw = (raw_data[8] << 8) | raw_data[9];
    int16_t gy_raw = (raw_data[10] << 8) | raw_data[11];
    int16_t gz_raw = (raw_data[12] << 8) | raw_data[13];

    // printf("RAW AX: %d AY: %d AZ: %d\n", ax_raw, ay_raw, az_raw);
    

    // convert accel
    float ax_f = ax_raw / ACCEL_SENS;
    float ay_f = ay_raw / ACCEL_SENS;
    float az_f = az_raw / ACCEL_SENS;

    // convert gyro
    float gx_f = gx_raw / GYRO_SENS;
    float gy_f = gy_raw / GYRO_SENS;
    float gz_f = gz_raw / GYRO_SENS;

    // Optional: apply scaling factor to Z accel to better match 1g at rest
    float az_scale = 0.91f;

    // APPLY BIAS CORRECTION (HERE)
    gx_f -= gx_bias;
    gy_f -= gy_bias;
    gz_f -= gz_bias;

    ax_f -= ax_bias;
    ay_f -= ay_bias;
    az_f -= az_bias;

    // Apply scaling factor to Z accel
    az_f *= az_scale;

    if (fabsf(gx_f) < GYRO_DEADBAND) gx_f = 0.0f;
    if (fabsf(gy_f) < GYRO_DEADBAND) gy_f = 0.0f;
    if (fabsf(gz_f) < GYRO_DEADBAND) gz_f = 0.0f;

    // Convert to physical units and store in pointers
    *ax = ax_f;
    *ay = ay_f;
    *az = az_f;

    *gx = gx_f;
    *gy = gy_f;
    *gz = gz_f;



    // ESP_LOGI(TAG, "ACCEL [g]: X=%.2f Y=%.2f Z=%.2f", accel_x, accel_y, accel_z);
    // ESP_LOGI(TAG, "GYRO [deg/s]: X=%.2f Y=%.2f Z=%.2f", gyro_x, gyro_y, gyro_z);

    // printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
    //    accel_x, accel_y, accel_z,
    //    gyro_x, gyro_y, gyro_z);
}

void imu_calibrate_gyro(void)
{
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    

    for (int i = 0; i < 500; i++) {
        float ax, ay, az, gx, gy, gz;

        imu_read(&ax, &ay, &az, &gx, &gy, &gz);

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    gx_bias = gx_sum / 500.0f;
    gy_bias = gy_sum / 500.0f;
    gz_bias = gz_sum / 500.0f;
}

void imu_calibrate_accel(void)
{
    float ax_sum = 0, ay_sum = 0, az_sum = 0;

    for (int i = 0; i < 500; i++) {
        float ax, ay, az, gx, gy, gz;
        imu_read(&ax, &ay, &az, &gx, &gy, &gz);

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ax_bias = ax_sum / 500.0f;
    ay_bias = ay_sum / 500.0f;
    az_bias = (az_sum / 500.0f) - 1.0f;
}