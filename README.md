# ESP-IDF C BNO08x IMU Driver

This project is inspired by [myles-parfeniuk/esp32_BNO08x](https://github.com/myles-parfeniuk/esp32_BNO08x), a C++ BNO08x driver.


# Examples

Reading data via polling with `BNO08x_data_available()`:
```cpp
#include <stdio.h>
#include "bno08x_driver.h"

void app_main(void)
{
    BNO08x imu;                               // imu object handle
    BNO08x_config_t cfg = DEFAULT_IMU_CONFIG; // default IMU config modifiable via idf.py menuconfig esp32_BNO08x

    // initialize SPI peripheral and gpio
    BNO08x_init(&imu, &cfg);
    // initialize BNO08x
    BNO08x_initialize(&imu);

    // enable reports for any data we want to receive
    BNO08x_enable_game_rotation_vector(&imu, 100000UL); // 100,000us == 100ms report interval
    BNO08x_enable_gyro(&imu, 150000UL);                 // 150,000us == 150ms report interval

    while (1)
    {
        //blocks until a valid packet is received or HOST_INT_TIMEOUT_MS occurs
        if (BNO08x_data_available(&imu))
        {
            float x, y, z = 0;

            // angular velocity (Rad/s)
            x = BNO08x_get_gyro_calibrated_velocity_X(&imu);
            y = BNO08x_get_gyro_calibrated_velocity_Y(&imu);
            z = BNO08x_get_gyro_calibrated_velocity_Z(&imu);
            ESP_LOGW("IMU_cb", "Angular Velocity: x: %.3f y: %.3f z: %.3f", x, y, z);

            // absolute heading (degrees)
            x = BNO08x_get_roll_deg(&imu);
            y = BNO08x_get_pitch_deg(&imu);
            z = BNO08x_get_yaw_deg(&imu);
            ESP_LOGI("IMU_cb", "Euler Angle: x (roll): %.3f y (pitch): %.3f z (yaw): %.3f", x, y, z);
        }
    }
}

```

Reading data with an automatically invoked callback using `BNO08x_register_cb()`:
```cpp
#include <stdio.h>
#include "bno08x_driver.h"

//imu data callback function, invoked whenever new data is arrived
void imu_data_cb(void *arg);

void app_main(void)
{
    BNO08x imu;                               // imu object handle
    BNO08x_config_t cfg = DEFAULT_IMU_CONFIG; // default IMU config modifiable via idf.py menuconfig esp32_BNO08x

    // initialize SPI peripheral and gpio
    BNO08x_init(&imu, &cfg);
    // initialize BNO08x
    BNO08x_initialize(&imu);

    // register a callback to be executed when new data is available
    BNO08x_register_cb(&imu, imu_data_cb);

    // enable reports for any data we want to receive
    BNO08x_enable_game_rotation_vector(&imu, 100000UL); // 100,000us == 100ms report interval
    BNO08x_enable_gyro(&imu, 150000UL);                 // 150,000us == 150ms report interval

    while (1)
    {
        // delay here is irrelevant, we just don't want cpu watchdog to trip
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void imu_data_cb(void *arg)
{
    BNO08x *imu = (BNO08x *)arg; //void pointer should always be casted to BNO08x struct pointer
    float x, y, z = 0;

    //angular velocity (Rad/s)
    x = BNO08x_get_gyro_calibrated_velocity_X(imu);
    y = BNO08x_get_gyro_calibrated_velocity_Y(imu);
    z = BNO08x_get_gyro_calibrated_velocity_Z(imu);
    ESP_LOGW("IMU_cb", "Angular Velocity: x: %.3f y: %.3f z: %.3f", x, y, z);

    //absolute heading (degrees)
    x = BNO08x_get_roll_deg(imu);
    y = BNO08x_get_pitch_deg(imu);
    z = BNO08x_get_yaw_deg(imu);
    ESP_LOGI("IMU_cb", "Euler Angle: x (roll): %.3f y (pitch): %.3f z (yaw): %.3f", x, y, z);
}

```