#include "imu_com_filter.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAU           1.0f
#define GYRO_DEADBAND 0.15f
#define ALPHA         0.95f

static float    roll_gyro  = 0.0f;
static float pitch_gyro = 0.0f;
static float roll_comp  = 0.0f;
static float pitch_comp = 0.0f;
static uint32_t last_time = 0;

void filter_init(void)
{
    roll_gyro  = 0.0f;
    pitch_gyro = 0.0f;
    roll_comp  = 0.0f;
    pitch_comp = 0.0f;
    last_time  = 0;
}

void filter_update(float ax, float ay, float az,
                   float gx, float gy, float gz,
                   float *out_roll_gyro,
                   float *out_roll_accel,
                   float *out_roll_comp,
                   float *out_pitch_gyro,
                   float *out_pitch_accel,
                   float *out_pitch_comp)
{
    // ---------------- DT ----------------
    uint32_t now = xTaskGetTickCount();
    float dt = (last_time == 0) ? 0.01f : (float)(now - last_time) / configTICK_RATE_HZ;
    last_time = now;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

    // ---------------- DEADBAND ----------------
    if (fabsf(gx) < GYRO_DEADBAND) gx = 0.0f;
    if (fabsf(gy) < GYRO_DEADBAND) gy = 0.0f;

    // ---------------- ACCEL ANGLES IN RADIANS ----------------
    float roll_acc  = atan2f(ay, az);
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

    // Convert to Degrees
    roll_acc  = roll_acc  * (180.0f / M_PI);
    pitch_acc = pitch_acc * (180.0f / M_PI);

    // ---------------- GYRO ONLY ----------------
    // gives pitch and roll in degrees
    roll_gyro  = roll_gyro  + gx * dt;
    pitch_gyro = pitch_gyro + gy * dt;

    // ---------------- COMPLEMENTARY FILTER ----------------
    roll_comp  = ALPHA * (roll_comp  + gx * dt) + (1.0f - ALPHA) * roll_acc;
    pitch_comp = ALPHA * (pitch_comp + gy * dt) + (1.0f - ALPHA) * pitch_acc;

    // ---------------- OUTPUT ----------------
    *out_roll_gyro  = roll_gyro;
    *out_pitch_gyro = pitch_gyro;

    *out_roll_accel  = roll_acc;
    *out_pitch_accel = pitch_acc;

    *out_roll_comp  = roll_comp;
    *out_pitch_comp = pitch_comp;
}