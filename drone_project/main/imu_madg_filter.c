#include "imu_madg_filter.h"
#include <math.h>

#define BETA 0.033f  // 6-axis default from the Python reference

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float beta = BETA;

void madgwick_init(void)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    beta = BETA;
}

void madgwick_update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt)
{
    // NOTE: gx/gy/gz must be in rad/s already.
    // Only uncomment the block below if your driver gives degrees/s.
    gx *= (M_PI / 180.0f);
    gy *= (M_PI / 180.0f);
    gz *= (M_PI / 180.0f);

    // --- Rate of change from gyroscope (pure integration term) ---
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // --- Gradient descent correction (accelerometer) ---
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0.0f) goto integrate; // skip correction, just integrate gyro
    ax /= norm;
    ay /= norm;
    az /= norm;

    {
        // Objective function f = R(q)*g_world - g_body
        // g_world = [0, 0, 1] in NED (gravity points down = +Z)
        // Matches Python: f[0]=2(qx*qz - qw*qy)-ax
        //                 f[1]=2(qw*qx + qy*qz)-ay
        //                 f[2]=2(0.5 - qx^2 - qy^2)-az
        float f0 = 2.0f*(q1*q3 - q0*q2) - ax;
        float f1 = 2.0f*(q0*q1 + q2*q3) - ay;
        float f2 = 2.0f*(0.5f - q1*q1 - q2*q2) - az;

        // Jacobian J^T * f  (J is 3x4, so J^T*f is 4x1)
        // Row 0 of J: [-2q2,  2q3, -2q0,  2q1]  <- df0/d[q0,q1,q2,q3]
        // Row 1 of J: [ 2q1,  2q0,  2q3,  2q2]  <- df1/d[q0,q1,q2,q3]
        // Row 2 of J: [   0, -4q1, -4q2,     0]  <- df2/d[q0,q1,q2,q3]
        float s0 = -2.0f*q2*f0 + 2.0f*q1*f1;
        float s1 =  2.0f*q3*f0 + 2.0f*q0*f1 - 4.0f*q1*f2;
        float s2 = -2.0f*q0*f0 + 2.0f*q3*f1 - 4.0f*q2*f2;
        float s3 =  2.0f*q1*f0 + 2.0f*q2*f1;

        // Normalize gradient
        norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (norm == 0.0f) goto integrate;
        s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm;

        // Subtract gradient correction from rate of change
        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;
    }

integrate:
    // Integrate quaternion rate
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // Normalize quaternion
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm == 0.0f) { madgwick_init(); return; } // safety reset
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

void madgwick_get_euler(float *roll, float *pitch, float *yaw)
{
    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * (180.0f / M_PI);
    *pitch = asinf (2.0f*(q0*q2 - q3*q1))                                * (180.0f / M_PI);
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * (180.0f / M_PI);
}