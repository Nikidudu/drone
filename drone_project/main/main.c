#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_data.h"
#include "imu_com_filter.h"

void app_main(void)
{
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch;

    imu_init();
    filter_init();
    imu_calibrate_gyro();

    while (1)
    {
        imu_read(&ax, &ay, &az, &gx, &gy, &gz);

        filter_update(ax, ay, az, gx, gy, gz, &roll, &pitch);

        // printf("%.2f,%.2f\n", roll, pitch);
        printf("%.2f,%.2f,%.2f\n", gx, gy, gz);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}