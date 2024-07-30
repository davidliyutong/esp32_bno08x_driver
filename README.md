# ESP-IDF C BNO08x IMU Driver

This project is inspired by [myles-parfeniuk/esp32_BNO08x](https://github.com/myles-parfeniuk/esp32_BNO08x), a C++ BNO08x driver.

Example

```c
#include <stdio.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include "driver/spi_master.h"
#include <esp_log.h>

#include <esp_rom_gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <rom/ets_sys.h>
#include "bno08x_driver.h"
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "example";

void app_main(void)
{
    BNO08x device;
    //BNO08x_config_t imu_config = DEFAULT_IMU_CONFIG;
    BNO08x_config_t imu_config={
    .spi_peripheral = SPI3_HOST,
    .io_mosi = GPIO_NUM_13, //assign pin
    .io_miso = GPIO_NUM_11, //assign pin
    .io_sclk = GPIO_NUM_12,
    .io_cs = GPIO_NUM_10, //assign pin
    .io_int = GPIO_NUM_9, //assign pin
    .io_rst = GPIO_NUM_8, //assign pin
    .io_wake = GPIO_NUM_14, //assign pin
    .sclk_speed = 200000 //assign pin
    };
    //imu_config.debug_en=1;
    //BNO08x imu(imu_config);  //pass config to BNO08x constructor
    
    BNO08x_init(&device,&imu_config);
    BNO08x_initialize(&device);
    BNO08x_enable_game_rotation_vector(&device,100000);
    BNO08x_enable_gyro(&device,150000);

    while (1)
    {
        if (BNO08x_data_available(&device))
        {
            ESP_LOGW("Main", "Velocity: x: %.3f y: %.3f z: %.3f", BNO08x_get_gyro_calibrated_velocity_X(&device), BNO08x_get_gyro_calibrated_velocity_Y(&device), BNO08x_get_gyro_calibrated_velocity_Z(&device));
            ESP_LOGI("Main", "Euler Angle: x (roll): %.3f y (pitch): %.3f z (yaw): %.3f", BNO08x_get_roll_deg(&device), BNO08x_get_pitch_deg(&device), BNO08x_get_yaw_deg(&device));
        }
        
    }
    

}

```

## add

CMakeLists.txt

```c
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")
```

