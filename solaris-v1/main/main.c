#include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
#include "pressure_sensor_driver.h" 
#include "imu_driver.h"



void app_main() {

    imu_init();
    pressure_sensor_init();

}