#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_data.h"

void app_main(void)
{
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch;

    imu_init();

    while (1)
    {
        imu_read(&ax, &ay, &az, &gx, &gy, &gz);

        filter_update(ax, ay, az, gx, gy, gz, &roll, &pitch);

        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               ax, ay, az, gx, gy, gz);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}