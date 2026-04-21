#include "imu_com_filter.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// filter tuning constant
#define TAU 1.0f
#define GYRO_DEADBAND 0.15f

static float roll = 0.0f;
static float pitch = 0.0f;
static uint32_t last_time = 0;



void filter_init(void)
{
    roll = 0.0f;
    pitch = 0.0f;
}


void filter_update(float ax, float ay, float az,
                   float gx, float gy, float gz,
                   float *out_roll, float *out_pitch)
{


    uint32_t now = xTaskGetTickCount();
    float dt = (last_time == 0) ? 0.01f : (float)(now - last_time) / configTICK_RATE_HZ;
    last_time = now;
    
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

    if (fabsf(gx) < GYRO_DEADBAND) gx = 0.0f;
    if (fabsf(gy) < GYRO_DEADBAND) gy = 0.0f;
    if (fabsf(gz) < GYRO_DEADBAND) gz = 0.0f;

    float ALPHA = TAU / (TAU + dt);

    float gyro_roll, gyro_pitch;

    // ---------------- ACCEL ANGLES ----------------
    float roll_acc  = atan2f(ay, az);
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

    // ---------------- GYRO INTEGRATION ----------------
    gyro_roll = roll + gx * dt;
    gyro_pitch = pitch + gy * dt;

    // ---------------- COMPLEMENTARY FILTER ----------------
    roll  = ALPHA * gyro_roll  + (1.0f - ALPHA) * roll_acc;
    pitch = ALPHA * gyro_pitch + (1.0f - ALPHA) * pitch_acc;

    // ---------------- OUTPUT ----------------
    *out_roll = roll;
    *out_pitch = pitch;
}
// The above code returns the angle in RADIANS