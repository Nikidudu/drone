#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_data.h"
#include "imu_com_filter.h"
#include "imu_madg_filter.h"

void app_main(void)
{
    float ax, ay, az;
    float gx, gy, gz;
    
    float roll_gyro,  pitch_gyro;
    float roll_accel, pitch_accel;
    float roll_comp,  pitch_comp, yaw_comp;

    imu_init();
    imu_calibrate_gyro();
    filter_init();
    madgwick_init();
    

    while (1)
    {
        imu_read(&ax, &ay, &az, &gx, &gy, &gz);

        // filter_update(ax, ay, az, gx, gy, gz,
        //               &roll_gyro, &roll_accel, &roll_comp,
        //               &pitch_gyro, &pitch_accel, &pitch_comp);

        madgwick_update(gx, gy, gz, ax, ay, az, 0.1f);
        madgwick_get_euler(&roll_comp, &pitch_comp, &yaw_comp);

        printf("%.2f,%.2f,%.2f\n", roll_comp, pitch_comp, yaw_comp);
            
       

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

