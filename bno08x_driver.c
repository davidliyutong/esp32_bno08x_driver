/*
 * This file is part of the [rfmarkit-esp-node].
 *
 * Original code from esp32_BNO08x (https://github.com/myles-parfeniuk/esp32_BNO08x)
 * by Myles Parfeniuk, licensed under the MIT License.
 *
 * Modifications by [davidliyutong], [2024].
*/
#include "bno08x_driver.h"

bool bno08x_isr_service_installed = false;

static const char *TAG = "imu[bno08x]";
/**
 * @brief BNO08x imu constructor.
 *
 * Construct a BNO08x object for managing a BNO08x sensor.
 * Initializes required GPIO pins, interrupts, SPI peripheral, and local task for SPI transactions.
 *
 * @param imu_config Configuration settings (optional), default settings can be seen in BNO08x_config_t
 * @return void, nothing to return
 */
void BNO08x_init(BNO08x* device, BNO08x_config_t *imu_config)
{
    device->int_asserted_semaphore = xSemaphoreCreateBinary();
    device->tx_semaphore = xSemaphoreCreateBinary();
    memcpy(&device->imu_config, imu_config, sizeof(BNO08x_config_t));
    device->calibration_status = 1;
    // clear all buffers
    memset(device->tx_buffer, 0, sizeof(device->tx_buffer));
    memset(device->rx_buffer, 0, sizeof(device->rx_buffer));
    memset(device->packet_header_rx, 0, sizeof(device->packet_header_rx));
    memset(device->commands, 0, sizeof(device->commands));
    memset(device->sequence_number, 0, sizeof(device->sequence_number));

    // SPI bus config
    device->bus_config.mosi_io_num = imu_config->io_mosi; // assign mosi gpio pin
    device->bus_config.miso_io_num = imu_config->io_miso; // assign miso gpio pin
    device->bus_config.sclk_io_num = imu_config->io_sclk; // assign sclk gpio pin
    device->bus_config.quadhd_io_num = -1;               // hold signal gpio (not used)
    device->bus_config.quadwp_io_num = -1;               // write protect signal gpio (not used)
    device->bus_config.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;

    // SPI slave device specific config
    device->imu_spi_config.mode = 0x3; // set mode to 3 as per BNO08x datasheet (CPHA second edge, CPOL bus high when idle)

    if (imu_config->sclk_speed > 3000000) // max sclk speed of 3MHz for BNO08x
    {
    ESP_LOGE(TAG, "Max clock speed exceeded, %d overwritten with 3000000Hz", imu_config->sclk_speed);
    imu_config->sclk_speed = 3000000;
    }

    // FIXME: device->imu_spi_config.clock_source = SPI_CLK_SRC_DEFAULT;
    device->imu_spi_config.clock_speed_hz = imu_config->sclk_speed; // assign SCLK speed
    device->imu_spi_config.address_bits = 0;                       // 0 address bits, not using this system
    device->imu_spi_config.command_bits = 0;                       // 0 command bits, not using this system
    device->imu_spi_config.spics_io_num = -1;                      // due to esp32 silicon issue, chip select cannot be used with full-duplex mode
    // driver, it must be handled via calls to gpio pins
    device->imu_spi_config.queue_size = 5;                         // only allow for 5 queued transactions at a timed

    // SPI non-driver-controlled GPIO config
    // configure outputs
    gpio_config_t outputs_config;

    if (imu_config->io_wake != GPIO_NUM_NC)
        outputs_config.pin_bit_mask = (1ULL << imu_config->io_cs) | (1ULL << imu_config->io_rst) | (1ULL << imu_config->io_wake); // configure CS, RST, and wake gpio pins
    else
        outputs_config.pin_bit_mask = (1ULL << imu_config->io_cs) | (1ULL << imu_config->io_rst);

    outputs_config.mode = GPIO_MODE_OUTPUT;
    outputs_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    outputs_config.pull_up_en = GPIO_PULLUP_DISABLE;
    outputs_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&outputs_config);
    gpio_set_level(imu_config->io_cs, 1);
    gpio_set_level(imu_config->io_rst, 1);

    if (imu_config->io_wake != GPIO_NUM_NC)
        gpio_set_level(imu_config->io_wake, 1);

    // configure input (HINT pin)
    gpio_config_t inputs_config;
    inputs_config.pin_bit_mask = (1ULL << imu_config->io_int);
    inputs_config.mode = GPIO_MODE_INPUT;
    inputs_config.pull_up_en = GPIO_PULLUP_ENABLE;
    inputs_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    inputs_config.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&inputs_config);

    // check if GPIO ISR service has been installed (only has to be done once regardless of SPI slaves being used)
    if (!bno08x_isr_service_installed)
    {
        gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); // install isr service
        bno08x_isr_service_installed = true;
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(imu_config->io_int, BNO08x_hint_handler, (void *)device));
    gpio_intr_disable(imu_config->io_int); // disable interrupts initially before reset

    // initialize the spi peripheral
    spi_bus_initialize(imu_config->spi_peripheral, &(device->bus_config), SPI_DMA_CH_AUTO);
    // add the imu device to the bus
    spi_bus_add_device(imu_config->spi_peripheral, &(device->imu_spi_config), &(device->spi_hdl));

    // do first SPI operation into nowhere before BNO085 reset to let periphiral stabilize (Anton B.)
    memset(device->tx_buffer, 0, sizeof(device->tx_buffer));
    device->spi_transaction.length = 8;
    device->spi_transaction.rxlength = 0;
    device->spi_transaction.tx_buffer = device->tx_buffer;
    device->spi_transaction.rx_buffer = NULL;
    device->spi_transaction.flags = 0;
    spi_device_polling_transmit(device->spi_hdl, &(device->spi_transaction)); // send data packet

    device->spi_task_hdl = NULL;
    xTaskCreate(&BNO08x_spi_task, "bno08x_spi_task", 4096, (void *)device, 8, &(device->spi_task_hdl)); // launch SPI task
}

/**
 * @brief Initializes BNO08x sensor
 *
 * Resets sensor and goes through initializing process outlined in BNO08x datasheet.
 *
 * @return void, nothing to return
 */
bool BNO08x_initialize(BNO08x* device)
{
    // Receive advertisement message on boot (see SH2 Ref. Manual 5.2 & 5.3)
    if (!BNO08x_hard_reset(device))
    {
        ESP_LOGE(TAG, "Failed to receive advertisement message on boot.");
        return false;
    }
    else
    {
        ESP_LOGI(TAG, "Received advertisement message.");
    }

    // The BNO080 will then transmit an unsolicited Initialize Response (see SH2 Ref. Manual 6.4.5.2)
    if (!BNO08x_wait_for_device_int(device))
    {
        ESP_LOGE(TAG, "Failed to receive initialize response on boot.");
        return false;
    }
    else
    {
        ESP_LOGI(TAG, "Received initialize response.");
    }

    // queue request for product ID command
    BNO08x_queue_request_product_id_command(device);
    // transmit request for product ID
    if (!BNO08x_wait_for_device_int(device))
    {
        ESP_LOGE(TAG, "Failed to send product ID report request");
        return false;
    }

    // receive product ID report
    if (!BNO08x_wait_for_device_int(device))
    {
        ESP_LOGE(TAG, "Failed to receive product ID report.");
        return false;
    }

    if (device->rx_buffer[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) // check to see that product ID matches what it should
    {
        uint32_t sw_part_number =
            ((uint32_t) device->rx_buffer[7] << 24) | ((uint32_t) device->rx_buffer[6] << 16) | ((uint32_t) device->rx_buffer[5] << 8) | ((uint32_t) device->rx_buffer[4]);
        uint32_t sw_build_number =
            ((uint32_t) device->rx_buffer[11] << 24) | ((uint32_t) device->rx_buffer[10] << 16) | ((uint32_t) device->rx_buffer[9] << 8) | ((uint32_t) device->rx_buffer[8]);
        uint16_t sw_version_patch = ((uint16_t) device->rx_buffer[13] << 8) | ((uint16_t) device->rx_buffer[12]);

        // print product ID info packet
        ESP_LOGI(TAG,
                 "Successfully initialized....\n\r"
                 "                ---------------------------\n\r"
                 "                Product ID: 0x%" PRIx32 "\n\r"
                 "                SW Version Major: 0x%" PRIx32 "\n\r"
                 "                SW Version Minor: 0x%" PRIx32 "\n\r"
                 "                SW Part Number:   0x%" PRIx32 "\n\r"
                 "                SW Build Number:  0x%" PRIx32 "\n\r"
                 "                SW Version Patch: 0x%" PRIx32 "\n\r"
                 "                ---------------------------\n\r",
            (uint32_t) device->rx_buffer[0], (uint32_t) device->rx_buffer[2], (uint32_t) device->rx_buffer[3], sw_part_number, sw_build_number,
            (uint32_t) sw_version_patch);
    }
    else
        return false;

    return true;
}

/**
 * @brief Re-enables interrupts and waits for BNO08x to assert HINT pin.
 *
 * @return void, nothing to return
 */
bool BNO08x_wait_for_device_int(BNO08x* device)
{
    gpio_intr_enable(device->imu_config.io_int); // re-enable interrupts

    // wait until an interrupt has been asserted or timeout has occured
    if (xSemaphoreTake(device->int_asserted_semaphore, HOST_INT_TIMEOUT_MS / portTICK_PERIOD_MS) == pdTRUE)
    {
        if (device->imu_config.debug_en)
            ESP_LOGI(TAG, "int asserted");

        return true;
    }
    else
    {
        ESP_LOGE(TAG, "Interrupt to host device never asserted.");
        return false;
    }
}

/**
 * @brief Hard resets BNO08x sensor.
 *
 * @return void, nothing to return
 */
bool BNO08x_hard_reset(BNO08x* device)
{
    gpio_set_level(device->imu_config.io_cs, 1);

    if (device->imu_config.io_wake != GPIO_NUM_NC)
        gpio_set_level(device->imu_config.io_wake, 1);

    gpio_set_level(device->imu_config.io_rst, 0); // set reset pin low
    vTaskDelay(50 / portTICK_PERIOD_MS);  // 10ns min, set to 50ms to let things stabilize(Anton)
    gpio_set_level(device->imu_config.io_rst, 1); // bring out of reset

    if (!BNO08x_wait_for_device_int(device)) // wait for BNO08x to assert INT pin
    {
        ESP_LOGE(TAG, "Reset Failed, interrupt to host device never asserted.");
        return false;
    }

    return true;
}

/**
 * @brief Soft resets BNO08x sensor using executable channel.
 *
 * @return True if reset was success.
 */
bool BNO08x_soft_reset(BNO08x* device)
{
    bool success = false;

    memset(device->commands, 0, sizeof(device->commands));
    device->commands[0] = 1;

    BNO08x_queue_packet(device, CHANNEL_EXECUTABLE, 1);
    success = BNO08x_wait_for_device_int(device);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    success = BNO08x_wait_for_device_int(device); // receive advertisement message;
    vTaskDelay(20 / portTICK_PERIOD_MS);
    success = BNO08x_wait_for_device_int(device); // receive initialize message

    return success;
}

/**
 * @brief Get the reason for the most recent reset.
 *
 * @return The reason for the most recent recent reset ( 1 = POR (power on reset), 2 = internal reset, 3 = watchdog
 * timer, 4 = external reset 5 = other)
 */
uint8_t BNO08x_get_reset_reason(BNO08x *device)
{
    memset(device->commands, 0, sizeof(device->commands));
    device->commands[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; // Request the product ID and reset info
    device->commands[1] = 0;                              // Reserved

    // Transmit packet on channel 2, 2 bytes
    BNO08x_queue_packet(device, CHANNEL_CONTROL, 2);
    BNO08x_wait_for_device_int(device);

    if (BNO08x_wait_for_device_int(device))
    {
        if (device->commands[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
            return (device->commands[1]);
    }

    return 0;
}

/**
 * @brief Turns on/ brings BNO08x sensor out of sleep mode using executable channel.
 *
 * @return True if exiting sleep mode was success.
 */
bool BNO08x_mode_on(BNO08x *device)
{
    bool success = false;

    memset(device->commands, 0, sizeof(device->commands));
    device->commands[0] = 2;

    BNO08x_queue_packet(device, CHANNEL_EXECUTABLE, 1);
    success = BNO08x_wait_for_device_int(device);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    success = BNO08x_wait_for_device_int(device); // receive advertisement message;
    vTaskDelay(20 / portTICK_PERIOD_MS);
    success = BNO08x_wait_for_device_int(device); // receive initialize message

    return success;
}

/**
 * @brief Puts BNO08x sensor into sleep/low power mode using executable channel.
 *
 * @return True if entering sleep mode was success.
 */
bool BNO08x_mode_sleep(BNO08x *device)
{
    bool success = false;

    memset(device->commands, 0, sizeof(device->commands));
    device->commands[0] = 3;

    BNO08x_queue_packet(device, CHANNEL_EXECUTABLE, 1);
    success = BNO08x_wait_for_device_int(device);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    success = BNO08x_wait_for_device_int(device); // receive advertisement message;
    vTaskDelay(20 / portTICK_PERIOD_MS);
    success = BNO08x_wait_for_device_int(device); // receive initialize message

    return success;
}

/**
 * @brief Receives a SHTP packet via SPI.
 *
 * @return void, nothing to return
 */
bool BNO08x_receive_packet(BNO08x* device)
{
    uint8_t dummy_header_tx[4];

    memset(device->packet_header_rx, 0, sizeof(device->packet_header_rx));
    memset(dummy_header_tx, 0, sizeof(dummy_header_tx));

    if (gpio_get_level(device->imu_config.io_int)) // ensure INT pin is low
        return false;

    // setup transaction to receive first 4 bytes (packet header)
    device->spi_transaction.rx_buffer = device->packet_header_rx;
    device->spi_transaction.tx_buffer = dummy_header_tx;
    device->spi_transaction.length = 4 * 8;
    device->spi_transaction.rxlength = 4 * 8;
    device->spi_transaction.flags = 0;

    gpio_set_level(device->imu_config.io_cs, 0);                    // assert chip select
    spi_device_polling_transmit(device->spi_hdl, &(device->spi_transaction)); // receive first 4 bytes (packet header)

    // calculate length of packet from received header
    device->packet_length_rx = (((uint16_t) device->packet_header_rx[1]) << 8) | ((uint16_t) device->packet_header_rx[0]);
    device->packet_length_rx &= ~(1 << 15); // Clear the MSbit

    if (device->imu_config.debug_en)
        ESP_LOGW(TAG, "packet rx length: %d", device->packet_length_rx);

    if (device->packet_length_rx == 0)
        return false;

    device->packet_length_rx -= 4; // remove 4 header bytes from packet length (we already read those)

    // setup transacton to read the data packet
    device->spi_transaction.rx_buffer = device->rx_buffer;
    device->spi_transaction.tx_buffer = NULL;
    device->spi_transaction.length = device->packet_length_rx * 8;
    device->spi_transaction.rxlength = device->packet_length_rx * 8;
    device->spi_transaction.flags = 0;

    spi_device_polling_transmit(device->spi_hdl, &(device->spi_transaction)); // receive rest of packet

    gpio_set_level(device->imu_config.io_cs, 1); // de-assert chip select

    return true;
}

/**
 * @brief Queues an SHTP packet to be sent via SPI.
 *
 * @return void, nothing to return
 */
void BNO08x_queue_packet(BNO08x* device, uint8_t channel_number, uint8_t data_length)
{
    device->packet_length_tx = data_length + 4; // add 4 bytes for header
    uint8_t i = 0;

    memset(device->tx_buffer, 0, sizeof(device->tx_buffer));

    device->tx_buffer[0] = device->packet_length_tx & 0xFF;           // packet length LSB
    device->tx_buffer[1] = device->packet_length_tx >> 8;             // packet length MSB
    device->tx_buffer[2] = channel_number;                    // channel number to write to
    device->tx_buffer[3] = device->sequence_number[channel_number]++; // increment and send sequence number (packet counter)

    // save commands to send to device->tx_buffer
    for (i = 0; i < data_length; i++)
    {
        device->tx_buffer[i + 4] = device->commands[i];
    }

    xSemaphoreGive(device->tx_semaphore);
}

/**
 * @brief Sends a queued SHTP packet via SPI.
 *
 * @return void, nothing to return
 */
void BNO08x_send_packet(BNO08x* device)
{
    // setup transaction to send packet
    device->spi_transaction.length = device->packet_length_tx * 8;
    device->spi_transaction.rxlength = 0;
    device->spi_transaction.tx_buffer = device->tx_buffer;
    device->spi_transaction.rx_buffer = NULL;
    device->spi_transaction.flags = 0;

    gpio_set_level(device->imu_config.io_cs, 0);                    // assert chip select
    spi_device_polling_transmit(device->spi_hdl, &(device->spi_transaction)); // send data packet

    gpio_set_level(device->imu_config.io_cs, 1); // de-assert chip select
}

/**
 * @brief Queues a packet containing a command.
 *
 * @param command The command to be sent.
 * @return void, nothing to return
 */
void BNO08x_queue_command(BNO08x* device, uint8_t command)
{
    device->commands[0] = SHTP_REPORT_COMMAND_REQUEST; // Command Request
    device->commands[1] = device->command_sequence_number++;   // Increments automatically each function call
    device->commands[2] = command;                     // Command

    BNO08x_queue_packet(device, CHANNEL_CONTROL, 12);
}

/**
 * @brief Queues a packet containing the request product ID command.
 *
 * @return void, nothing to return
 */
void BNO08x_queue_request_product_id_command(BNO08x* device)
{
    device->commands[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; // request product ID and reset info
    device->commands[1] = 0;                              // reserved
    BNO08x_queue_packet(device, CHANNEL_CONTROL, 2);
}

/**
 * @brief Sends command to calibrate accelerometer, gyro, and magnetometer.
 *
 * @return void, nothing to return
 */
void BNO08x_calibrate_all(BNO08x* device)
{
    BNO08x_queue_calibrate_command(device, CALIBRATE_ACCEL_GYRO_MAG);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate accelerometer.
 *
 * @return void, nothing to return
 */
void BNO08x_calibrate_accelerometer(BNO08x* device)
{
    BNO08x_queue_calibrate_command(device, CALIBRATE_ACCEL);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate gyro.
 *
 * @return void, nothing to return
 */
void BNO08x_calibrate_gyro(BNO08x* device)
{
    BNO08x_queue_calibrate_command(device, CALIBRATE_GYRO);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate magnetometer.
 *
 * @return void, nothing to return
 */
void BNO08x_calibrate_magnetometer(BNO08x* device)
{
    BNO08x_queue_calibrate_command(device, CALIBRATE_MAG);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to calibrate planar accelerometer
 *
 * @return void, nothing to return
 */
void BNO08x_calibrate_planar_accelerometer(BNO08x* device)
{
    BNO08x_queue_calibrate_command(device, CALIBRATE_PLANAR_ACCEL);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Queues a packet containing a command to calibrate the specified sensor.
 *
 * @param sensor_to_calibrate The sensor to calibrate.
 * @return void, nothing to return
 */
void BNO08x_queue_calibrate_command(BNO08x* device, uint8_t sensor_to_calibrate)
{
    memset(device->commands, 0, sizeof(device->commands));

    switch (sensor_to_calibrate)
    {
        case CALIBRATE_ACCEL:
            device->commands[3] = 1;
            break;

        case CALIBRATE_GYRO:
            device->commands[4] = 1;
            break;

        case CALIBRATE_MAG:
            device->commands[5] = 1;
            break;

        case CALIBRATE_PLANAR_ACCEL:
            device->commands[7] = 1;
            break;

        case CALIBRATE_ACCEL_GYRO_MAG:
            device->commands[3] = 1;
            device->commands[4] = 1;
            device->commands[5] = 1;
            break;

        case CALIBRATE_STOP:
            // do nothing, send packet of all 0s
            break;

        default:

            break;
    }

    device->calibration_status = 1;

    BNO08x_queue_command(device, COMMAND_ME_CALIBRATE);
}

/**
 * @brief Requests ME calibration status from BNO08x (see Ref. Manual 6.4.7.2)
 *
 * @return void, nothing to return
 */
void BNO08x_request_calibration_status(BNO08x* device)
{
    memset(device->commands, 0, sizeof(device->commands));

    device->commands[6] = 0x01; // P3 - 0x01 - Subcommand: Get ME Calibration

    // Using this commands packet, send a command
    BNO08x_queue_command(device, COMMAND_ME_CALIBRATE);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Returns true if calibration has completed.
 *
 * @return void, nothing to return
 */
bool BNO08x_calibration_complete(BNO08x* device)
{
    if (device->calibration_status == 0)
        return true;

    return false;
}

/**
 * @brief Sends command to end calibration procedure.
 *
 * @return void, nothing to return
 */
void BNO08x_end_calibration(BNO08x* device)
{
    BNO08x_queue_calibrate_command(device, CALIBRATE_STOP); // Disables all calibrations
    BNO08x_wait_for_device_int(device);                   // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS);     // allow some time for command to be executed
}

/**
 * @brief Sends command to save internal calibration data (See Ref. Manual 6.4.7).
 *
 * @return void, nothing to return
 */
void BNO08x_save_calibration(BNO08x* device)
{
    memset(device->commands, 0, sizeof(device->commands));

    // Using this shtpData packet, send a command
    BNO08x_queue_command(device, COMMAND_DCD);          // Save DCD command
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Runs full calibration routine.
 *
 * Enables game rotation vector and magnetometer, starts ME calibration process.
 * Waits for accuracy of returned quaternions and magnetic field vectors to be high, then saves calibration data and
 * returns.
 *
 * @return void, nothing to return
 */
bool BNO08x_run_full_calibration_routine(BNO08x* device)
{
    float magf_x = 0;
    float magf_y = 0;
    float magf_z = 0;
    uint8_t magnetometer_accuracy = (uint8_t) IMU_ACCURACY_LOW;

    float quat_I = 0;
    float quat_J = 0;
    float quat_K = 0;
    float quat_real = 0;
    uint8_t quat_accuracy = (uint8_t) IMU_ACCURACY_LOW;

    uint16_t high_accuracy = 0;
    uint16_t save_calibration_attempt = 0;

    // Enable dynamic calibration for accel, gyro, and mag
    BNO08x_calibrate_all(device); // Turn on cal for Accel, Gyro, and Mag

    // Enable Game Rotation Vector output
    BNO08x_enable_game_rotation_vector(device, 100); // Send data update every 100ms

    // Enable Magnetic Field output
    BNO08x_enable_magnetometer(device, 100); // Send data update every 100ms

    while (1)
    {
        if (BNO08x_data_available(device))
        {
            magf_x = BNO08x_get_magf_X(device);
            magf_y = BNO08x_get_magf_Y(device);
            magf_z = BNO08x_get_magf_Z(device);
            magnetometer_accuracy = BNO08x_get_magf_accuracy(device);

            quat_I = BNO08x_get_quat_I(device);
            quat_J = BNO08x_get_quat_J(device);
            quat_K = BNO08x_get_quat_K(device);
            quat_real = BNO08x_get_quat_real(device);
            quat_accuracy = BNO08x_get_quat_accuracy(device);

            ESP_LOGI(TAG, "Magnetometer: x: %.3f y: %.3f z: %.3f, accuracy: %d", magf_x, magf_y, magf_z, magnetometer_accuracy);
            ESP_LOGI(TAG, "Quaternion Rotation Vector: i: %.3f j: %.3f k: %.3f, real: %.3f, accuracy: %d", quat_I, quat_J, quat_K, quat_real,
                     quat_accuracy);

            vTaskDelay(5 / portTICK_PERIOD_MS);

            if ((magnetometer_accuracy >= (uint8_t) IMU_ACCURACY_MED) && (quat_accuracy == (uint8_t) IMU_ACCURACY_HIGH))
                high_accuracy++;
            else
                high_accuracy = 0;

            if (high_accuracy > 10)
            {
                BNO08x_save_calibration(device);
                BNO08x_request_calibration_status(device);

                save_calibration_attempt = 0;

                while (save_calibration_attempt < 20)
                {
                    if (BNO08x_data_available(device))
                    {
                        if (BNO08x_calibration_complete(device))
                        {
                            ESP_LOGW(TAG, "Calibration data successfully stored.");
                            return true;
                        }
                        else
                        {
                            BNO08x_save_calibration(device);
                            BNO08x_request_calibration_status(device);
                            save_calibration_attempt++;
                        }
                    }
                }

                vTaskDelay(1 / portTICK_PERIOD_MS);

                if (save_calibration_attempt >= 20)
                    ESP_LOGE(TAG, "Calibration data failed to store.");

                return false;
            }
        }

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Checks if BNO08x has asserted interrupt and sent data.
 *
 * @return true if new data has been parsed and saved
 */
bool BNO08x_data_available(BNO08x* device)
{
    return (BNO08x_get_readings(device) != 0);
}

/**
 * @brief Waits for BNO08x HINT pin to assert, and parses the received data.
 *
 * @return void, nothing to return
 */
uint16_t BNO08x_get_readings(BNO08x* device)
{
    if (BNO08x_wait_for_device_int(device))
    {
        if (device->imu_config.debug_en)
            ESP_LOGE(
                TAG, "SHTP Header RX'd: 0x%X 0x%X 0x%X 0x%X", device->packet_header_rx[0], device->packet_header_rx[1], device->packet_header_rx[2], device->packet_header_rx[3]);

        // Check to see if this packet is a sensor reporting its data to us
        if (device->packet_header_rx[2] == CHANNEL_REPORTS && device->rx_buffer[0] == SHTP_REPORT_BASE_TIMESTAMP)
        {
            if (device->imu_config.debug_en)
                ESP_LOGI(TAG, "RX'd packet, channel report");

            return BNO08x_parse_input_report(device); // This will update the rawAccelX, etc variables depending on which feature
            // report is found
        }
        else if (device->packet_header_rx[2] == CHANNEL_CONTROL)
        {
            if (device->imu_config.debug_en)
                ESP_LOGI(TAG, "RX'd packet, channel control");

            return BNO08x_parse_command_report(device); // This will update responses to commands, calibrationStatus, etc.
        }
        else if (device->packet_header_rx[2] == CHANNEL_GYRO)
        {
            if (device->imu_config.debug_en)
                ESP_LOGI(TAG, "Rx packet, channel gyro");

            return BNO08x_parse_input_report(device); // This will update the rawAccelX, etc variables depending on which feature
            // report is found
        }
    }

    return 0;
}

/**
 * @brief Parses received input report sent by BNO08x.
 *
 * Unit responds with packet that contains the following:
 *
 * packet_header_rx[0:3]: First, a 4 byte header
 * rx_buffer[0:4]: Then a 5 byte timestamp of microsecond ticks since reading was taken
 * rx_buffer[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector, etc...)
 * rx_buffer[5 + 1]: Sequence number (See Ref.Manual 6.5.8.2)
 * rx_buffer[5 + 2]: Status
 * rx_buffer[3]: Delay
 * rx_buffer[4:5]: i/accel x/gyro x/etc
 * rx_buffer[6:7]: j/accel y/gyro y/etc
 * rx_buffer[8:9]: k/accel z/gyro z/etc
 * rx_buffer[10:11]: real/gyro temp/etc
 * rx_buffer[12:13]: Accuracy estimate
 *
 * @return void, nothing to return
 */
uint16_t BNO08x_parse_input_report(BNO08x* device)
{
    uint16_t i = 0;
    uint8_t status = 0;
    uint8_t command = 0;

    // Calculate the number of data bytes in this packet
    uint16_t data_length = ((uint16_t) device->packet_header_rx[1] << 8 | device->packet_header_rx[0]);
    data_length &= ~(1 << 15); // Clear the MSbit. This bit indicates if this package is a continuation of the last.
    // Ignore it for now. TODO catch this as an error and exit

    data_length -= 4; // Remove the header bytes from the data count
    device->time_stamp = ((uint32_t) device->rx_buffer[4] << (8 * 3)) | ((uint32_t) device->rx_buffer[3] << (8 * 2)) | ((uint32_t) device->rx_buffer[2] << (8 * 1)) |
                 ((uint32_t) device->rx_buffer[1] << (8 * 0));

    // The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence,
    // and status fields
    if (device->packet_header_rx[2] == CHANNEL_GYRO)
    {
        device->raw_quat_I = (uint16_t) device->rx_buffer[1] << 8 | device->rx_buffer[0];
        device->raw_quat_J = (uint16_t) device->rx_buffer[3] << 8 | device->rx_buffer[2];
        device->raw_quat_K = (uint16_t) device->rx_buffer[5] << 8 | device->rx_buffer[4];
        device->raw_quat_real = (uint16_t) device->rx_buffer[7] << 8 | device->rx_buffer[6];
        device->raw_velocity_gyro_X = (uint16_t) device->rx_buffer[9] << 8 | device->rx_buffer[8];
        device->raw_velocity_gyro_Y = (uint16_t) device->rx_buffer[11] << 8 | device->rx_buffer[10];
        device->raw_velocity_gyro_Z = (uint16_t) device->rx_buffer[13] << 8 | device->rx_buffer[12];

        return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
    }

    status = device->rx_buffer[5 + 2] & 0x03; // Get status bits
    uint16_t data1 = (uint16_t) device->rx_buffer[5 + 5] << 8 | device->rx_buffer[5 + 4];
    uint16_t data2 = (uint16_t) device->rx_buffer[5 + 7] << 8 | device->rx_buffer[5 + 6];
    uint16_t data3 = (uint16_t) device->rx_buffer[5 + 9] << 8 | device->rx_buffer[5 + 8];
    uint16_t data4 = 0;
    uint16_t data5 = 0;
    uint16_t data6 = 0;

    if (data_length - 5 > 9)
    {
        data4 = (uint16_t) device->rx_buffer[5 + 11] << 8 | device->rx_buffer[5 + 10];
    }
    if (data_length - 5 > 11)
    {
        data5 = (uint16_t) device->rx_buffer[5 + 13] << 8 | device->rx_buffer[5 + 12];
    }
    if (data_length - 5 > 13)
    {
        data6 = (uint16_t) device->rx_buffer[5 + 15] << 8 | device->rx_buffer[5 + 14];
    }

    // Store these generic values to their proper global variable
    switch (device->rx_buffer[5])
    {
        case SENSOR_REPORTID_ACCELEROMETER:
            device->accel_accuracy = status;
            device->raw_accel_X = data1;
            device->raw_accel_Y = data2;
            device->raw_accel_Z = data3;
            break;

        case SENSOR_REPORTID_LINEAR_ACCELERATION:
            device->accel_lin_accuracy = status;
            device->raw_lin_accel_X = data1;
            device->raw_lin_accel_Y = data2;
            device->raw_lin_accel_Z = data3;
            break;

        case SENSOR_REPORTID_GYROSCOPE:
            device->gyro_accuracy = status;
            device->raw_gyro_X = data1;
            device->raw_gyro_Y = data2;
            device->raw_gyro_Z = data3;
            break;

        case SENSOR_REPORTID_UNCALIBRATED_GYRO:
            device->uncalib_gyro_accuracy = status;
            device->raw_uncalib_gyro_X = data1;
            device->raw_uncalib_gyro_Y = data2;
            device->raw_uncalib_gyro_Z = data3;
            device->raw_bias_X = data4;
            device->raw_bias_Y = data5;
            device->raw_bias_Z = data6;
            break;

        case SENSOR_REPORTID_MAGNETIC_FIELD:
            device->magf_accuracy = status;
            device->raw_magf_X = data1;
            device->raw_magf_Y = data2;
            device->raw_magf_Z = data3;
            break;

        case SENSOR_REPORTID_TAP_DETECTOR:
            device->tap_detector = device->rx_buffer[5 + 4]; // Byte 4 only
            break;

        case SENSOR_REPORTID_STEP_COUNTER:
            device->step_count = data3; // Bytes 8/9
            break;

        case SENSOR_REPORTID_STABILITY_CLASSIFIER:
            device->stability_classifier = device->rx_buffer[5 + 4]; // Byte 4 only
            break;

        case SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER:
            device->activity_classifier = device->rx_buffer[5 + 5]; // Most likely state

            // Load activity classification confidences into the array
            for (i = 0; i < 9; i++)                             // Hardcoded to max of 9. TODO - bring in array size
                device->activity_confidences[i] = device->rx_buffer[5 + 6 + i]; // 5 bytes of timestamp, byte 6 is first confidence
            // byte
            break;

        case SENSOR_REPORTID_RAW_ACCELEROMETER:
            device->mems_raw_accel_X = data1;
            device->mems_raw_accel_Y = data2;
            device->mems_raw_accel_Z = data3;
            break;

        case SENSOR_REPORTID_RAW_GYROSCOPE:
            device->mems_raw_gyro_X = data1;
            device->mems_raw_gyro_Y = data2;
            device->mems_raw_gyro_Z = data3;
            break;

        case SENSOR_REPORTID_RAW_MAGNETOMETER:
            device->mems_raw_magf_X = data1;
            device->mems_raw_magf_Y = data2;
            device->mems_raw_magf_Z = data3;
            break;

        case SHTP_REPORT_COMMAND_RESPONSE:
            // The BNO080 responds with this report to command requests. It's up to use to remember which command we
            // issued.
            command = device->rx_buffer[5 + 2]; // This is the Command byte of the response

            if (command == COMMAND_ME_CALIBRATE)
                device->calibration_status = device->rx_buffer[5 + 5]; // R0 - Status (0 = success, non-zero = fail)
            break;

        case SENSOR_REPORTID_GRAVITY:
            device->gravity_accuracy = status;
            device->gravity_X = data1;
            device->gravity_Y = data2;
            device->gravity_Z = data3;
            break;

        default:
            if (device->rx_buffer[5] == SENSOR_REPORTID_ROTATION_VECTOR || device->rx_buffer[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
                device->rx_buffer[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
                device->rx_buffer[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR)
            {
                device->quat_accuracy = status;
                device->raw_quat_I = data1;
                device->raw_quat_J = data2;
                device->raw_quat_K = data3;
                device->raw_quat_real = data4;

                // Only available on rotation vector and ar/vr stabilized rotation vector,
                //  not game rot vector and not ar/vr stabilized rotation vector
                device->raw_quat_radian_accuracy = data5;
            }
            else
            {
                // This sensor report ID is unhandled.
                // See reference manual to add additional feature reports as needed
                return 0;
            }

            break;
    }

    // TODO additional feature reports may be strung together. Parse them all.
    return device->rx_buffer[5];
}

/**
 * @brief Parses received command report sent by BNO08x (See Ref. Manual 6.3.9)
 *
 * @return The command report ID, 0 if invalid.
 */
uint16_t BNO08x_parse_command_report(BNO08x* device)
{
    uint8_t command = 0;

    if (device->rx_buffer[0] == SHTP_REPORT_COMMAND_RESPONSE)
    {
        // The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
        command = device->rx_buffer[2]; // This is the Command byte of the response

        if (command == COMMAND_ME_CALIBRATE)
        {
            device->calibration_status = device->rx_buffer[5 + 0]; // R0 - Status (0 = success, non-zero = fail)
        }
        return device->rx_buffer[0];
    }
    else
    {
        // This sensor report ID is unhandled.
        // See SH2 Ref. Manual to add additional feature reports as needed
    }

    return 0;
}

/**
 * @brief Sends command to enable game rotation vector reports (See Ref. Manual 6.5.19)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_game_rotation_vector(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_GAME_ROTATION_VECTOR, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable rotation vector reports (See Ref. Manual 6.5.18)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_rotation_vector(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_ROTATION_VECTOR, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable ARVR stabilized rotation vector reports (See Ref. Manual 6.5.42)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_ARVR_stabilized_rotation_vector(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable ARVR stabilized game rotation vector reports (See Ref. Manual 6.5.43)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_ARVR_stabilized_game_rotation_vector(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable gyro integrated rotation vector reports (See Ref. Manual 6.5.44)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_gyro_integrated_rotation_vector(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable accelerometer reports (See Ref. Manual 6.5.9)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_accelerometer(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_ACCELEROMETER, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable linear accelerometer reports (See Ref. Manual 6.5.10)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_linear_accelerometer(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_LINEAR_ACCELERATION, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable gravity reading reports (See Ref. Manual 6.5.11)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_gravity(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_GRAVITY, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable gyro reports (See Ref. Manual 6.5.13)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_gyro(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_GYROSCOPE, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable uncalibrated gyro reports (See Ref. Manual 6.5.14)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_uncalibrated_gyro(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_UNCALIBRATED_GYRO, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable magnetometer reports (See Ref. Manual 6.5.16)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_magnetometer(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_MAGNETIC_FIELD, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable tap detector reports (See Ref. Manual 6.5.27)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_tap_detector(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_TAP_DETECTOR, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable step counter reports (See Ref. Manual 6.5.29)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_step_counter(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_STEP_COUNTER, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable activity stability classifier reports (See Ref. Manual 6.5.31)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_stability_classifier(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_STABILITY_CLASSIFIER, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable activity classifier reports (See Ref. Manual 6.5.36)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 *  @param activities_to_enable Desired activities to enable (0x1F enables all).
 *  @param activity_confidence_vals Returned activity level confidences.
 * @return void, nothing to return
 */
void BNO08x_enable_activity_classifier(BNO08x* device, uint32_t time_between_reports, uint32_t activities_to_enable, uint8_t activity_confidence_vals[9])
{
    device->activity_confidences = activity_confidence_vals; // Store pointer to array
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, time_between_reports, activities_to_enable);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable raw accelerometer reports (See Ref. Manual 6.5.8)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_raw_accelerometer(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_RAW_ACCELEROMETER, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable raw gyro reports (See Ref. Manual 6.5.12)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_raw_gyro(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_RAW_GYROSCOPE, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to enable raw magnetometer reports (See Ref. Manual 6.5.15)
 *
 * @param time_between_reports Desired time between reports in microseconds.
 * @return void, nothing to return
 */
void BNO08x_enable_raw_magnetometer(BNO08x* device, uint32_t time_between_reports)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_RAW_MAGNETOMETER, time_between_reports, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable rotation vector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_rotation_vector(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_ROTATION_VECTOR, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable game rotation vector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_game_rotation_vector(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_GAME_ROTATION_VECTOR, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable ARVR stabilized rotation vector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_ARVR_stabilized_rotation_vector(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable ARVR stabilized game rotation vector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_ARVR_stabilized_game_rotation_vector(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable gyro integrated rotation vector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_gyro_integrated_rotation_vector(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_ROTATION_VECTOR, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable accelerometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_accelerometer(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_ACCELEROMETER, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable linear accelerometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_linear_accelerometer(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_LINEAR_ACCELERATION, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable gravity reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_gravity(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_GRAVITY, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable gyro reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_gyro(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_GYROSCOPE, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable uncalibrated gyro reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_uncalibrated_gyro(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_UNCALIBRATED_GYRO, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable magnetometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_magnetometer(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_MAGNETIC_FIELD, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable tap detector reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_tap_detector(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_TAP_DETECTOR, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable step counter reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_step_counter(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_STEP_COUNTER, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable stability reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_stability_classifier(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_STABILITY_CLASSIFIER, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable activity classifier reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_activity_classifier(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable raw accelerometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_raw_accelerometer(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_RAW_ACCELEROMETER, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable raw gyro reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_raw_gyro(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_RAW_GYROSCOPE, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to disable raw magnetometer reports by setting report interval to 0.
 *
 * @return void, nothing to return
 */
void BNO08x_disable_raw_magnetometer(BNO08x* device)
{
    BNO08x_queue_feature_command(device, SENSOR_REPORTID_RAW_MAGNETOMETER, 0, 0);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(50 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to tare an axis (See Ref. Manual 6.4.4.1)
 *
 * @param axis_sel Which axes to zero, can be TARE_AXIS_ALL (all axes) or TARE_AXIS_Z (only yaw)
 * @param rotation_vector_basis Which rotation vector type to zero axes can be TARE_ROTATION_VECTOR,
 * TARE_GAME_ROTATION_VECTOR, TARE_GEOMAGNETIC_ROTATION_VECTOR, etc..
 * @return void, nothing to return
 */
void BNO08x_tare_now(BNO08x* device, uint8_t axis_sel, uint8_t rotation_vector_basis)
{
    BNO08x_queue_tare_command(device, TARE_NOW, axis_sel, rotation_vector_basis);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(12 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to save tare into non-volatile memory of BNO08x (See Ref. Manual 6.4.4.2)
 *
 * @return void, nothing to return
 */
void BNO08x_save_tare(BNO08x* device)
{
    BNO08x_queue_tare_command(device, TARE_PERSIST, TARE_AXIS_ALL, TARE_ROTATION_VECTOR);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(12 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Sends command to clear persistent tare settings in non-volatile memory of BNO08x (See Ref. Manual 6.4.4.3)
 *
 * @return void, nothing to return
 */
void BNO08x_clear_tare(BNO08x* device)
{
    BNO08x_queue_tare_command(device, TARE_SET_REORIENTATION, TARE_AXIS_ALL, TARE_ROTATION_VECTOR);
    BNO08x_wait_for_device_int(device);               // wait for next interrupt such that command is sent
    vTaskDelay(12 / portTICK_PERIOD_MS); // allow some time for command to be executed
}

/**
 * @brief Converts a register value to a float using its associated Q point. (See
 * https://en.wikipedia.org/wiki/Q_(number_format))
 *
 * @param q_point Q point value associated with register.
 * @param fixed_point_value The fixed point value to convert.
 *
 * @return void, nothing to return
 */
float BNO08x_q_to_float(int16_t fixed_point_value, uint8_t q_point)
{
    float q_float = fixed_point_value;
    q_float *= powf(2, (float)q_point * -1);
    return (q_float);
}

/**
 * @brief Return timestamp of most recent report.
 *
 * @return void, nothing to return
 */
uint32_t BNO08x_get_time_stamp(BNO08x* device)
{
    return device->time_stamp;
}

/**
 * @brief Get the full magnetic field vector.
 *
 * @param x Reference variable to save reported x magnitude.
 * @param y Reference variable to save reported y magnitude.
 * @param x Reference variable to save reported z magnitude.
 * @param accuracy Reference variable save reported accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x_get_magf(BNO08x* device, float* x, float* y, float* z, uint8_t* accuracy)
{
    *x = BNO08x_q_to_float(device->raw_magf_X, MAGNETOMETER_Q1);
    *y = BNO08x_q_to_float(device->raw_magf_Y, MAGNETOMETER_Q1);
    *z = BNO08x_q_to_float(device->raw_magf_Z, MAGNETOMETER_Q1);
    *accuracy = device->magf_accuracy;
}

/**
 * @brief Get X component of magnetic field vector.
 *
 * @return The reported X component of magnetic field vector.
 */
float BNO08x_get_magf_X(BNO08x* device)
{
    float mag = BNO08x_q_to_float(device->raw_magf_X, MAGNETOMETER_Q1);
    return mag;
}

/**
 * @brief Get Y component of magnetic field vector.
 *
 * @return The reported Y component of magnetic field vector.
 */
float BNO08x_get_magf_Y(BNO08x* device)
{
    float mag = BNO08x_q_to_float(device->raw_magf_Y, MAGNETOMETER_Q1);
    return mag;
}

/**
 * @brief Get Z component of magnetic field vector.
 *
 * @return The reported Z component of magnetic field vector.
 */
float BNO08x_get_magf_Z(BNO08x* device)
{
    float mag = BNO08x_q_to_float(device->raw_magf_Z, MAGNETOMETER_Q1);
    return mag;
}

/**
 * @brief Get accuracy of reported magnetic field vector.
 *
 * @return The accuracy of reported magnetic field vector.
 */
uint8_t BNO08x_get_magf_accuracy(BNO08x* device)
{
    return device->magf_accuracy;
}

/**
 * @brief Get full reported gravity vector, units in m/s^2
 *
 * @param x Reference variable to save X axis gravity.
 * @param y Reference variable to save Y axis axis gravity.
 * @param z Reference variable to save Z axis axis gravity.
 * @param accuracy Reference variable to save reported gravity accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x_get_gravity(BNO08x* device, float* x, float* y, float* z, uint8_t* accuracy)
{
    *x = BNO08x_q_to_float(device->gravity_X, GRAVITY_Q1);
    *y = BNO08x_q_to_float(device->gravity_Y, GRAVITY_Q1);
    *z = BNO08x_q_to_float(device->gravity_Z, GRAVITY_Q1);
    *accuracy = device->gravity_accuracy;
}

/**
 * @brief Get the reported x axis gravity.
 *
 * @return x axis gravity in m/s^2
 */
float BNO08x_get_gravity_X(BNO08x* device)
{
    return BNO08x_q_to_float(device->gravity_X, GRAVITY_Q1);
}

/**
 * @brief Get the reported y axis gravity.
 *
 * @return y axis gravity in m/s^2
 */
float BNO08x_get_gravity_Y(BNO08x* device)
{
    return BNO08x_q_to_float(device->gravity_Y, GRAVITY_Q1);
}

/**
 * @brief Get the reported z axis gravity.
 *
 * @return z axis gravity in m/s^2
 */
float BNO08x_get_gravity_Z(BNO08x* device)
{
    return BNO08x_q_to_float(device->gravity_Z, GRAVITY_Q1);
}

/**
 * @brief Get the reported gravity accuracy.
 *
 * @return Accuracy of reported gravity.
 */
uint8_t BNO08x_get_gravity_accuracy(BNO08x* device)
{
    return device->gravity_accuracy;
}

/**
 * @brief Get the reported rotation about x axis.
 *
 * @return Rotation about the x axis in radians.
 */
float BNO08x_get_roll(BNO08x* device)
{
    float t_0 = 0.0f;
    float t_1 = 0.0f;
    float dq_w = BNO08x_get_quat_real(device);
    float dq_x = BNO08x_get_quat_I(device);
    float dq_y = BNO08x_get_quat_J(device);
    float dq_z = BNO08x_get_quat_K(device);

    float norm = sqrtf(dq_w * dq_w + dq_x * dq_x + dq_y * dq_y + dq_z * dq_z);
    dq_w = dq_w / norm;
    dq_x = dq_x / norm;
    dq_y = dq_y / norm;
    dq_z = dq_z / norm;

    // roll (x-axis rotation)
    t_0 = 2.0f * (dq_w * dq_x + dq_y * dq_z);
    t_1 = 1.0f - (2.0f * ((dq_x * dq_x) + (dq_y * dq_y)));

    return atan2f(t_0, t_1);
}

/**
 * @brief Get the reported rotation about y axis.
 *
 * @return Rotation about the y axis in radians.
 */
float BNO08x_get_pitch(BNO08x* device)
{
    float t_2 = 0.0f;
    float dq_w = BNO08x_get_quat_real(device);
    float dq_x = BNO08x_get_quat_I(device);
    float dq_y = BNO08x_get_quat_J(device);
    float dq_z = BNO08x_get_quat_K(device);
    float norm = sqrtf(dq_w * dq_w + dq_x * dq_x + dq_y * dq_y + dq_z * dq_z);

    dq_w = dq_w / norm;
    dq_x = dq_x / norm;
    dq_y = dq_y / norm;
    dq_z = dq_z / norm;

    // float ysqr = dqy * dqy;

    // pitch (y-axis rotation)
    t_2 = 2.0f * ((dq_w * dq_y) - (dq_z * dq_x));
    t_2 = t_2 > 1.0 ? 1.0f : t_2;
    t_2 = t_2 < -1.0 ? -1.0f : t_2;

    return asinf(t_2);
}

/**
 * @brief Get the reported rotation about z axis.
 *
 * @return Rotation about the z axis in radians.
 */
float BNO08x_get_yaw(BNO08x* device)
{
    float t_3 = 0.0f;
    float t_4 = 0.0f;
    float dq_w = BNO08x_get_quat_real(device);
    float dq_x = BNO08x_get_quat_I(device);
    float dq_y = BNO08x_get_quat_J(device);
    float dq_z = BNO08x_get_quat_K(device);
    float norm = sqrtf(dq_w * dq_w + dq_x * dq_x + dq_y * dq_y + dq_z * dq_z);

    dq_w = dq_w / norm;
    dq_x = dq_x / norm;
    dq_y = dq_y / norm;
    dq_z = dq_z / norm;

    // yaw (z-axis rotation)
    t_3 = 2.0f * ((dq_w * dq_z) + (dq_x * dq_y));
    t_4 = 1.0f - (2.0 * ((dq_y * dq_y) + (dq_z * dq_z)));

    return atan2f(t_3, t_4);
}

/**
 * @brief Get the reported rotation about x axis.
 *
 * @return Rotation about the x axis in degrees.
 */
float BNO08x_get_roll_deg(BNO08x* device)
{
    return BNO08x_get_roll(device) * (float)(180.0f / M_PI);
}

/**
 * @brief Get the reported rotation about y axis.
 *
 * @return Rotation about the y axis in degrees.
 */
float BNO08x_get_pitch_deg(BNO08x* device)
{
    return BNO08x_get_pitch(device) * (float)(180.0f / M_PI);
}

/**
 * @brief Get the reported rotation about z axis.
 *
 * @return Rotation about the z axis in degrees.
 */
float BNO08x_get_yaw_deg(BNO08x* device)
{
    return BNO08x_get_yaw(device) * (float)(180.0 / M_PI);
}

/**
 * @brief Get the full quaternion reading.
 *
 * @param i Reference variable to save reported i component of quaternion.
 * @param j Reference variable to save reported j component of quaternion.
 * @param k Reference variable to save reported k component of quaternion.
 * @param real Reference variable to save reported real component of quaternion.
 * @param rad_accuracy Reference variable to save reported raw quaternion radian accuracy.
 * @param accuracy Reference variable to save reported quaternion accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x_get_quat(BNO08x* device, float* i, float* j, float* k, float* real, float* rad_accuracy, uint8_t* accuracy)
{
    *i = BNO08x_q_to_float(device->raw_quat_I, ROTATION_VECTOR_Q1);
    *j = BNO08x_q_to_float(device->raw_quat_J, ROTATION_VECTOR_Q1);
    *k = BNO08x_q_to_float(device->raw_quat_K, ROTATION_VECTOR_Q1);
    *real = BNO08x_q_to_float(device->raw_quat_real, ROTATION_VECTOR_Q1);
    *rad_accuracy = BNO08x_q_to_float(device->raw_quat_radian_accuracy, ROTATION_VECTOR_Q1);
    *accuracy = device->quat_accuracy;
}

/**
 * @brief Get I component of reported quaternion.
 *
 * @return The I component of reported quaternion.
 */
float BNO08x_get_quat_I(BNO08x* device)
{
    float quat = BNO08x_q_to_float(device->raw_quat_I, ROTATION_VECTOR_Q1);
    return quat;
}

/**
 * @brief Get J component of reported quaternion.
 *
 * @return The J component of reported quaternion.
 */
float BNO08x_get_quat_J(BNO08x* device)
{
    float quat = BNO08x_q_to_float(device->raw_quat_J, ROTATION_VECTOR_Q1);
    return quat;
}

/**
 * @brief Get K component of reported quaternion.
 *
 * @return The K component of reported quaternion.
 */
float BNO08x_get_quat_K(BNO08x* device)
{
    float quat = BNO08x_q_to_float(device->raw_quat_K, ROTATION_VECTOR_Q1);
    return quat;
}

/**
 * @brief Get real component of reported quaternion.
 *
 * @return The real component of reported quaternion.
 */
float BNO08x_get_quat_real(BNO08x* device)
{
    float quat = BNO08x_q_to_float(device->raw_quat_real, ROTATION_VECTOR_Q1);
    return quat;
}

/**
 * @brief Get radian accuracy of reported quaternion.
 *
 * @return The radian accuracy of reported quaternion.
 */
float BNO08x_get_quat_radian_accuracy(BNO08x* device)
{
    float quat = BNO08x_q_to_float(device->raw_quat_radian_accuracy, ROTATION_VECTOR_Q1);
    return quat;
}

/**
 * @brief Get accuracy of reported quaternion.
 *
 * @return The accuracy of reported quaternion.
 */
uint8_t BNO08x_get_quat_accuracy(BNO08x* device)
{
    return device->quat_accuracy;
}

/**
 * @brief Get full acceleration (total acceleration of device, units in m/s^2).
 *
 * @param x Reference variable to save X axis acceleration.
 * @param y Reference variable to save Y axis acceleration.
 * @param z Reference variable to save Z axis acceleration.
 * @param accuracy Reference variable to save reported acceleration accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x_get_accel(BNO08x* device, float* x, float* y, float* z, uint8_t* accuracy)
{
    *x = BNO08x_q_to_float(device->raw_accel_X, ACCELEROMETER_Q1);
    *y = BNO08x_q_to_float(device->raw_accel_Y, ACCELEROMETER_Q1);
    *z = BNO08x_q_to_float(device->raw_accel_Z, ACCELEROMETER_Q1);
    *accuracy = device->accel_accuracy;
}

/**
 * @brief Get x axis acceleration (total acceleration of device, units in m/s^2).
 *
 * @return The angular reported x axis acceleration.
 */
float BNO08x_get_accel_X(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_accel_X, ACCELEROMETER_Q1);
}

/**
 * @brief Get y axis acceleration (total acceleration of device, units in m/s^2).
 *
 * @return The angular reported y axis acceleration.
 */
float BNO08x_get_accel_Y(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_accel_Y, ACCELEROMETER_Q1);
}

/**
 * @brief Get z axis acceleration (total acceleration of device, units in m/s^2).
 *
 * @return The angular reported z axis acceleration.
 */
float BNO08x_get_accel_Z(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_accel_Z, ACCELEROMETER_Q1);
}

/**
 * @brief Get accuracy of linear acceleration.
 *
 * @return Accuracy of linear acceleration.
 */
uint8_t BNO08x_get_accel_accuracy(BNO08x* device)
{
    return device->accel_accuracy;
}

/**
 * @brief Get full linear acceleration (acceleration of the device minus gravity, units in m/s^2).
 *
 * @param x Reference variable to save X axis acceleration.
 * @param y Reference variable to save Y axis acceleration.
 * @param z Reference variable to save Z axis acceleration.
 * @param accuracy Reference variable to save reported linear acceleration accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x_get_linear_accel(BNO08x* device, float* x, float* y, float* z, uint8_t* accuracy)
{
    *x = BNO08x_q_to_float(device->raw_lin_accel_X, LINEAR_ACCELEROMETER_Q1);
    *y = BNO08x_q_to_float(device->raw_lin_accel_Y, LINEAR_ACCELEROMETER_Q1);
    *z = BNO08x_q_to_float(device->raw_lin_accel_Z, LINEAR_ACCELEROMETER_Q1);
    *accuracy = device->accel_lin_accuracy;
}

/**
 * @brief Get x axis linear acceleration (acceleration of device minus gravity, units in m/s^2)
 *
 * @return The angular reported x axis linear acceleration.
 */
float BNO08x_get_linear_accel_X(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_lin_accel_X, LINEAR_ACCELEROMETER_Q1);
}

/**
 * @brief Get y axis linear acceleration (acceleration of device minus gravity, units in m/s^2)
 *
 * @return The angular reported y axis linear acceleration.
 */
float BNO08x_get_linear_accel_Y(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_lin_accel_Y, LINEAR_ACCELEROMETER_Q1);
}

/**
 * @brief Get z axis linear acceleration (acceleration of device minus gravity, units in m/s^2)
 *
 * @return The angular reported z axis linear acceleration.
 */
float BNO08x_get_linear_accel_Z(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_lin_accel_Z, LINEAR_ACCELEROMETER_Q1);
}

/**
 * @brief Get accuracy of linear acceleration.
 *
 * @return Accuracy of linear acceleration.
 */
uint8_t BNO08x_get_linear_accel_accuracy(BNO08x* device)
{
    return device->accel_lin_accuracy;
}

/**
 * @brief Get raw accelerometer x axis reading from physical accelerometer MEMs sensor (See Ref. Manual 6.5.8)
 *
 * @return Reported raw accelerometer x axis reading from physical MEMs sensor.
 */
int16_t BNO08x_get_raw_accel_X(BNO08x* device)
{
    return device->mems_raw_accel_X;
}

/**
 * @brief Get raw accelerometer y axis reading from physical accelerometer MEMs sensor (See Ref. Manual 6.5.8)
 *
 * @return Reported raw accelerometer y axis reading from physical MEMs sensor.
 */
int16_t BNO08x_get_raw_accel_Y(BNO08x* device)
{
    return device->mems_raw_accel_Y;
}

/**
 * @brief Get raw accelerometer z axis reading from physical accelerometer MEMs sensor (See Ref. Manual 6.5.8)
 *
 * @return Reported raw accelerometer z axis reading from physical MEMs sensor.
 */
int16_t BNO08x_get_raw_accel_Z(BNO08x* device)
{
    return device->mems_raw_accel_Z;
}

/**
 * @brief Get raw gyroscope x axis reading from physical gyroscope MEMs sensor (See Ref. Manual 6.5.12)
 *
 * @return Reported raw gyroscope x axis reading from physical MEMs sensor.
 */
int16_t BNO08x_get_raw_gyro_X(BNO08x* device)
{
    return device->mems_raw_gyro_X;
}

/**
 * @brief Get raw gyroscope y axis reading from physical gyroscope MEMs sensor (See Ref. Manual 6.5.12)
 *
 * @return Reported raw gyroscope y axis reading from physical MEMs sensor.
 */
int16_t BNO08x_get_raw_gyro_Y(BNO08x* device)
{
    return device->mems_raw_gyro_Y;
}

/**
 * @brief Get raw gyroscope z axis reading from physical gyroscope MEMs sensor (See Ref. Manual 6.5.12)
 *
 * @return Reported raw gyroscope z axis reading from physical MEMs sensor.
 */
int16_t BNO08x_get_raw_gyro_Z(BNO08x* device)
{
    return device->mems_raw_gyro_Z;
}

/**
 * @brief Get raw magnetometer x axis reading from physical magnetometer sensor (See Ref. Manual 6.5.15)
 *
 * @return Reported raw magnetometer x axis reading from physical magnetometer sensor.
 */
int16_t BNO08x_get_raw_magf_X(BNO08x* device)
{
    return device->mems_raw_magf_X;
}

/**
 * @brief Get raw magnetometer y axis reading from physical magnetometer sensor (See Ref. Manual 6.5.15)
 *
 * @return Reported raw magnetometer y axis reading from physical magnetometer sensor.
 */
int16_t BNO08x_get_raw_magf_Y(BNO08x* device)
{
    return device->mems_raw_magf_Y;
}

/**
 * @brief Get raw magnetometer z axis reading from physical magnetometer sensor (See Ref. Manual 6.5.15)
 *
 * @return Reported raw magnetometer z axis reading from physical magnetometer sensor.
 */
int16_t BNO08x_get_raw_magf_Z(BNO08x* device)
{
    return device->mems_raw_magf_Z;
}

/**
 * @brief Get full rotational velocity with drift compensation (units in Rad/s).
 *
 * @param x Reference variable to save X axis angular velocity
 * @param y Reference variable to save Y axis angular velocity
 * @param z Reference variable to save Z axis angular velocity
 * @param accuracy Reference variable to save reported gyro accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x_get_gyro_calibrated_velocity(BNO08x* device, float* x, float* y, float* z, uint8_t* accuracy)
{
    *x = BNO08x_q_to_float(device->raw_gyro_X, GYRO_Q1);
    *y = BNO08x_q_to_float(device->raw_gyro_Y, GYRO_Q1);
    *z = BNO08x_q_to_float(device->raw_gyro_Z, GYRO_Q1);
    *accuracy = device->gyro_accuracy;
}

/**
 * @brief Get calibrated gyro x axis angular velocity measurement.
 *
 * @return The angular reported x axis angular velocity from calibrated gyro (drift compensation applied).
 */
float BNO08x_get_gyro_calibrated_velocity_X(BNO08x* device)
{
    return BNO08x_q_to_float((int16_t) device->raw_gyro_X, GYRO_Q1);
}

/**
 * @brief Get calibrated gyro y axis angular velocity measurement.
 *
 * @return The angular reported y axis angular velocity from calibrated gyro (drift compensation applied).
 */
float BNO08x_get_gyro_calibrated_velocity_Y(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_gyro_Y, GYRO_Q1);
}

/**
 * @brief Get calibrated gyro z axis angular velocity measurement.
 *
 * @return The angular reported z axis angular velocity from calibrated gyro (drift compensation applied).
 */
float BNO08x_get_gyro_calibrated_velocity_Z(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_gyro_Z, GYRO_Q1);
}

/**
 * @brief Get calibrated gyro accuracy.
 *
 * @return Accuracy of calibrated gyro.
 */
uint8_t BNO08x_get_gyro_accuracy(BNO08x* device)
{
    return device->gyro_accuracy;
}

/**
 * @brief Get full rotational velocity without drift compensation (units in Rad/s). An estimate of drift is given but
 * not applied.
 *
 * @param x Reference variable to save X axis angular velocity
 * @param y Reference variable to save Y axis angular velocity
 * @param z Reference variable to save Z axis angular velocity
 * @param b_x Reference variable to save X axis drift estimate
 * @param b_y Reference variable to save Y axis drift estimate
 * @param b_z Reference variable to save Z axis drift estimate
 * @param accuracy Reference variable to save reported gyro accuracy.
 *
 * @return void, nothing to return
 */
void BNO08x_get_uncalibrated_gyro(BNO08x* device, float* x, float* y, float* z, float* b_x, float* b_y, float* b_z, uint8_t* accuracy)
{
    *x = BNO08x_q_to_float(device->raw_uncalib_gyro_X, GYRO_Q1);
    *y = BNO08x_q_to_float(device->raw_uncalib_gyro_Y, GYRO_Q1);
    *z = BNO08x_q_to_float(device->raw_uncalib_gyro_Z, GYRO_Q1);
    *b_x = BNO08x_q_to_float(device->raw_bias_X, GYRO_Q1);
    *b_y = BNO08x_q_to_float(device->raw_bias_Y, GYRO_Q1);
    *b_z = BNO08x_q_to_float(device->raw_bias_Z, GYRO_Q1);
    *accuracy = device->uncalib_gyro_accuracy;
}

/**
 * @brief Get uncalibrated gyro x axis angular velocity measurement.
 *
 * @return The angular reported x axis angular velocity from uncalibrated gyro.
 */
float BNO08x_get_uncalibrated_gyro_X(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_uncalib_gyro_X, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro Y axis angular velocity measurement.
 *
 * @return The angular reported Y axis angular velocity from uncalibrated gyro.
 */
float BNO08x_get_uncalibrated_gyro_Y(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_uncalib_gyro_Y, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro Z axis angular velocity measurement.
 *
 * @return The angular reported Z axis angular velocity from uncalibrated gyro.
 */
float BNO08x_get_uncalibrated_gyro_Z(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_uncalib_gyro_Z, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro x axis drift estimate.
 *
 * @return The angular reported x axis drift estimate.
 */
float BNO08x_get_uncalibrated_gyro_bias_X(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_bias_X, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro Y axis drift estimate.
 *
 * @return The angular reported Y axis drift estimate.
 */
float BNO08x_get_uncalibrated_gyro_bias_Y(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_bias_Y, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro Z axis drift estimate.
 *
 * @return The angular reported Z axis drift estimate.
 */
float BNO08x_get_uncalibrated_gyro_bias_Z(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_bias_Z, GYRO_Q1);
}

/**
 * @brief Get uncalibrated gyro accuracy.
 *
 * @return Accuracy of uncalibrated gyro.
 */
uint8_t BNO08x_get_uncalibrated_gyro_accuracy(BNO08x* device)
{
    return device->uncalib_gyro_accuracy;
}

/**
 * @brief Full rotational velocity from gyro-integrated rotation vector (See Ref. Manual 6.5.44)
 *
 * @param x Reference variable to save X axis angular velocity
 * @param y Reference variable to save Y axis angular velocity
 * @param z Reference variable to save Z axis angular velocity
 *
 * @return void, nothing to return
 */
void BNO08x_get_gyro_velocity(BNO08x* device, float* x, float* y, float* z)
{
    *x = BNO08x_q_to_float(device->raw_velocity_gyro_X, ANGULAR_VELOCITY_Q1);
    *y = BNO08x_q_to_float(device->raw_velocity_gyro_Y, ANGULAR_VELOCITY_Q1);
    *z = BNO08x_q_to_float(device->raw_velocity_gyro_Z, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get x axis angular velocity from gyro-integrated rotation vector. (See Ref. Manual 6.5.44)
 *
 * @return The reported x axis angular velocity.
 */
float BNO08x_get_gyro_velocity_X(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_velocity_gyro_X, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get y axis angular velocity from gyro-integrated rotation vector. (See Ref. Manual 6.5.44)
 *
 * @return The reported y axis angular velocity.
 */
float BNO08x_get_gyro_velocity_Y(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_velocity_gyro_Y, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get z axis angular velocity from gyro-integrated rotation vector. (See Ref. Manual 6.5.44)
 *
 * @return The reported Z axis angular velocity.
 */
float BNO08x_get_gyro_velocity_Z(BNO08x* device)
{
    return BNO08x_q_to_float(device->raw_velocity_gyro_Z, ANGULAR_VELOCITY_Q1);
}

/**
 * @brief Get if tap has occured.
 *
 * @return 7 bit tap code indicated which axis taps have occurred. (See Ref. Manual 6.5.27)
 */
uint8_t BNO08x_get_tap_detector(BNO08x* device)
{
    uint8_t previous_tap_detector = device->tap_detector;
    device->tap_detector = 0; // Reset so user code sees exactly one tap
    return (previous_tap_detector);
}

/**
 * @brief Get the counted amount of steps.
 *
 * @return The current amount of counted steps.
 */
uint16_t BNO08x_get_step_count(BNO08x* device)
{
    return device->step_count;
}

/**
 * @brief Get the current stability classifier (Seee Ref. Manual 6.5.31)
 *
 * @return The current stability (0 = unknown, 1 = on table, 2 = stationary)
 */
int8_t BNO08x_get_stability_classifier(BNO08x* device)
{
    return device->stability_classifier;
}

/**
 * @brief Get the current activity classifier (Seee Ref. Manual 6.5.36)
 *
 * @return The current activity:
 *         0 = unknown
 *         1 = in vehicle
 *         2 = on bicycle
 *         3 = on foot
 *         4 = still
 *         5 = tilting
 *         6 = walking
 *         7 = runnning
 *         8 = on stairs
 */
uint8_t BNO08x_get_activity_classifier(BNO08x* device)
{
    return device->activity_classifier;
}

/**
 * @brief Prints the most recently received SHTP header to serial console with ESP_LOG statement.
 *
 * @return void, nothing to return
 */
void BNO08x_print_header(BNO08x* device)
{
    // print most recent header
    ESP_LOGI(TAG,
             "SHTP Header:\n\r"
             "                       Raw 32 bit word: 0x%02X%02X%02X%02X\n\r"
             "                       Packet Length:   %d\n\r"
             "                       Channel Number:  %d\n\r"
             "                       Sequence Number: %d\n\r"
             "                       Channel Type: %s\n\r",
             (int) device->packet_header_rx[0], (int) device->packet_header_rx[1], (int) device->packet_header_rx[2], (int) device->packet_header_rx[3], (int) (device->packet_length_rx + 4),
             (int) device->packet_header_rx[2], (int) device->packet_header_rx[3],
             (device->packet_header_rx[2] == 0)   ? "Command"
                                          : (device->packet_header_rx[2] == 1) ? "Executable"
                                                                       : (device->packet_header_rx[2] == 2) ? "Control"
                                                                                                    : (device->packet_header_rx[2] == 3) ? "Sensor-report"
                                                                                                                                 : (device->packet_header_rx[2] == 4) ? "Wake-report"
                                                                                                                                                              : (device->packet_header_rx[2] == 5) ? "Gyro-vector"
                                                                                                                                                                                           : "Unknown");
}

void BNO08x_print_packet(BNO08x* device)
{
    uint8_t i = 0;
    uint16_t print_length = 0;
    char packet_string[600];
    char byte_string[8];

    if (device->packet_length_rx > 40)
        print_length = 40;
    else
        print_length = device->packet_length_rx;

    sprintf(packet_string, "                 Body: \n\r                       ");
    for (i = 0; i < print_length; i++)
    {
        sprintf(byte_string, " 0x%02X ", device->rx_buffer[i]);
        strcat(packet_string, byte_string);

        if ((i + 1) % 6 == 0) // add a newline every 6 bytes
            strcat(packet_string, "\n\r                       ");
    }

    ESP_LOGI(TAG,
             "SHTP Header:\n\r"
             "                       Raw 32 bit word: 0x%02X%02X%02X%02X\n\r"
             "                       Packet Length:   %d\n\r"
             "                       Channel Number:  %d\n\r"
             "                       Sequence Number: %d\n\r"
             "                       Channel Type: %s\n\r"
             "%s",
             (int) device->packet_header_rx[0], (int) device->packet_header_rx[1], (int) device->packet_header_rx[2], (int) device->packet_header_rx[3], (int) (device->packet_length_rx + 4),
             (int) device->packet_header_rx[2], (int) device->packet_header_rx[3],
             (device->packet_header_rx[2] == 0)   ? "Command"
                                          : (device->packet_header_rx[2] == 1) ? "Executable"
                                                                       : (device->packet_header_rx[2] == 2) ? "Control"
                                                                                                    : (device->packet_header_rx[2] == 3) ? "Sensor-report"
                                                                                                                                 : (device->packet_header_rx[2] == 4) ? "Wake-report"
                                                                                                                                                              : (device->packet_header_rx[2] == 5) ? "Gyro-vector"
                                                                                                                                                                                           : "Unknown",
             packet_string);
}

/**
 * @brief Gets Q1 point from BNO08x FRS (flash record system).
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to get Q1 value for.
 *
 * @return Q1 value for requested sensor.
 */
int16_t BNO08x_get_Q1(BNO08x* device, uint16_t record_ID)
{
    // Q1 is lower 16 bits of word 7
    return (uint16_t) (BNO08x_FRS_read_word(device, record_ID, 7) & 0xFFFF);
}

/**
 * @brief Gets Q2 point from BNO08x FRS (flash record system).
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to get Q2 value for.
 *
 * @return Q2 value for requested sensor.
 */
int16_t BNO08x_get_Q2(BNO08x* device, uint16_t record_ID)
{
    // Q2 is upper 16 bits of word 7
    return (uint16_t) (BNO08x_FRS_read_word(device, record_ID, 7) >> 16U);
}

/**
 * @brief Gets Q3 point from BNO08x FRS (flash record system).
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to get Q3 value for.
 *
 * @return Q3 value for requested sensor.
 */
int16_t BNO08x_get_Q3(BNO08x* device, uint16_t record_ID)
{
    // Q3 is upper 16 bits of word 8
    return (uint16_t) (BNO08x_FRS_read_word(device, record_ID, 8) >> 16U);
}

/**
 * @brief Gets resolution from BNO08x FRS (flash record system).
 *
 * @param record_ID Which record ID/ sensor to get resolution value for.
 *
 * @return The resolution value for the requested sensor.
 */
float BNO08x_get_resolution(BNO08x* device, uint16_t record_ID)
{
    int16_t Q = BNO08x_get_Q1(device, record_ID); // use same q as sensor's input report for range calc

    // resolution is word 2
    uint32_t value = BNO08x_FRS_read_word(device, record_ID, 2);

    return BNO08x_q_to_float(value, Q); // return resolution
}

/**
 * @brief Gets range from BNO08x FRS (flash record system).
 *
 * @param record_ID Which record ID/ sensor to get range value for.
 *
 * @return The range value for the requested sensor.
 */
float BNO08x_get_range(BNO08x* device, uint16_t record_ID)
{
    int16_t Q = BNO08x_get_Q1(device, record_ID); // use same q as sensor's input report for range calc

    // resolution is word 1
    uint32_t value = BNO08x_FRS_read_word(device, record_ID, 1);

    return BNO08x_q_to_float(value, Q); // return range
}

/**
 * @brief Reads meta data word from BNO08x FRS (flash record system) given the record ID and word number. (See Ref.
 * Manual 5.1 & 6.3.7)
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to request meta data from.
 * @param word_number Desired word to read.
 *
 * @return Requested meta data word, 0 if failed.
 */
uint32_t BNO08x_FRS_read_word(BNO08x* device, uint16_t record_ID, uint8_t word_number)
{
    if (BNO08x_FRS_read_data(device, record_ID, word_number, 1)) // start at desired word and only read one 1 word
        return (device->meta_data[0]);
    else
        return 0; // FRS read failed
}

/**
 * @brief Requests meta data from BNO08x FRS (flash record system) given the record ID. Contains Q points and other
 * info. (See Ref. Manual 5.1 & 6.3.6)
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to request meta data from.
 * @param start_location Start byte location.
 * @param words_to_read Length of words to read.
 *
 * @return True if read request acknowledged (HINT was asserted)
 */
bool BNO08x_FRS_read_request(BNO08x* device, uint16_t record_ID, uint16_t read_offset, uint16_t block_size)
{
    memset(device->commands, 0, sizeof(device->commands));

    device->commands[0] = SHTP_REPORT_FRS_READ_REQUEST; // FRS Read Request
    device->commands[1] = 0;                            // Reserved
    device->commands[2] = (read_offset >> 0) & 0xFF;    // Read Offset LSB
    device->commands[3] = (read_offset >> 8) & 0xFF;    // Read Offset MSB
    device->commands[4] = (record_ID >> 0) & 0xFF;      // FRS Type LSB
    device->commands[5] = (record_ID >> 8) & 0xFF;      // FRS Type MSB
    device->commands[6] = (block_size >> 0) & 0xFF;     // Block size LSB
    device->commands[7] = (block_size >> 8) & 0xFF;     // Block size MSB

    // Transmit packet on channel 2, 8 bytes
    BNO08x_queue_packet(device, CHANNEL_CONTROL, 8);
    return BNO08x_wait_for_device_int(device);
}

/**
 * @brief Read meta data from BNO08x FRS (flash record system) given the record ID. Contains Q points and other info.
 * (See Ref. Manual 5.1 & 6.3.7)
 *
 * Note that Q points from the data sheet can be used as well, using the ones stored in flash is optional.
 *
 * @param record_ID Which record ID/ sensor to request meta data from.
 * @param start_location Start byte location.
 * @param words_to_read Length of words to read.
 *
 * @return True if meta data read successfully.
 */
bool BNO08x_FRS_read_data(BNO08x* device, uint16_t record_ID, uint8_t start_location, uint8_t words_to_read)
{
    uint32_t data_0 = 0;
    uint32_t data_1 = 0;
    uint8_t data_length = 0;
    uint8_t FRS_status = 0;
    uint8_t attempt_count = 0;
    uint16_t i = 0;

    if (BNO08x_FRS_read_request(device,record_ID, start_location, words_to_read))
    {
        vTaskDelay(5 / portTICK_PERIOD_MS);
        for (attempt_count = 0; attempt_count < 10; attempt_count++)
        {
            if (BNO08x_wait_for_device_int(device))
            {
                if (device->rx_buffer[0] != SHTP_REPORT_FRS_READ_RESPONSE)
                    return false;

                if (((((uint16_t) device->rx_buffer[13]) << 8) | device->rx_buffer[12]) != record_ID)
                    return false;
            }

            data_length = device->rx_buffer[1] >> 4;
            FRS_status = device->rx_buffer[1] & 0x0F;

            data_0 = (uint32_t) device->rx_buffer[7] << 24 | (uint32_t) device->rx_buffer[6] << 16 | (uint32_t) device->rx_buffer[5] << 8 | (uint32_t) device->rx_buffer[4];
            data_1 = (uint32_t) device->rx_buffer[11] << 24 | (uint32_t) device->rx_buffer[10] << 16 | (uint32_t) device->rx_buffer[9] << 8 | (uint32_t) device->rx_buffer[8];

            // save meta data words to their respective buffer
            if (data_length > 0)
                device->meta_data[i++] = data_0;

            if (data_length > 1)
                device->meta_data[i++] = data_1;

            if (i >= 9)
            {
                if (device->imu_config.debug_en)
                    ESP_LOGW(TAG, "meta_data array overrun, returning from FRS_read_request()");

                return true;
            }

            if (FRS_status == 3 || FRS_status == 6 || FRS_status == 7)
                return true;
        }
    }

    return false;
}

/**
 * @brief Queues a packet containing a command with a request for sensor reports, reported periodically. (See Ref.
 * Manual 6.5.4)
 *
 * @param report_ID ID of sensor report to be enabled.
 * @param time_between_reports Desired time between reports in microseconds.
 * @param specific_config Specific config word (used with personal activity classifier)
 *
 * @return void, nothing to return
 */
void BNO08x_queue_feature_command(BNO08x* device, uint8_t report_ID, uint32_t time_between_reports, uint32_t specific_config)
{
    memset(device->commands, 0, sizeof(device->commands));

    device->commands[0] = SHTP_REPORT_SET_FEATURE_COMMAND;     // Set feature command (See Ref. Manual 6.5.4)
    device->commands[1] = report_ID;                           // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    device->commands[2] = 0;                                   // Feature flags
    device->commands[3] = 0;                                   // Change sensitivity (LSB)
    device->commands[4] = 0;                                   // Change sensitivity (MSB)
    device->commands[5] = (time_between_reports >> 0) & 0xFF;  // Report interval (LSB) in microseconds. 0x7A120 = 500ms
    device->commands[6] = (time_between_reports >> 8) & 0xFF;  // Report interval
    device->commands[7] = (time_between_reports >> 16) & 0xFF; // Report interval
    device->commands[8] = (time_between_reports >> 24) & 0xFF; // Report interval (MSB)
    device->commands[9] = 0;                                   // Batch Interval (LSB)
    device->commands[10] = 0;                                  // Batch Interval
    device->commands[11] = 0;                                  // Batch Interval
    device->commands[12] = 0;                                  // Batch Interval (MSB)
    device->commands[13] = (specific_config >> 0) & 0xFF;      // Sensor-specific config (LSB)
    device->commands[14] = (specific_config >> 8) & 0xFF;      // Sensor-specific config
    device->commands[15] = (specific_config >> 16) & 0xFF;     // Sensor-specific config
    device->commands[16] = (specific_config >> 24) & 0xFF;     // Sensor-specific config (MSB)

    // Transmit packet on channel 2, 17 bytes
    BNO08x_queue_packet(device, CHANNEL_CONTROL, 17);
}

/**
 * @brief Queues a packet containing a command related to zeroing sensor's axes. (See Ref. Manual 6.4.4.1)
 *
 * @param command Tare command to be sent.
 * @param axis Specified axis (can be z or all at once)
 * @param rotation_vector_basis Which rotation vector type to zero axes of, BNO08x saves seperate data for Rotation
 * Vector, Gaming Rotation Vector, etc..)
 *
 * @return void, nothing to return
 */
void BNO08x_queue_tare_command(BNO08x* device, uint8_t command, uint8_t axis, uint8_t rotation_vector_basis)
{
    memset(device->commands, 0, sizeof(device->commands));

    device->commands[3] = command;

    if (command == TARE_NOW)
    {
        device->commands[4] = axis;
        device->commands[5] = rotation_vector_basis;
    }

    BNO08x_queue_command(device, COMMAND_TARE);
}

/**
 * @brief Queues a packet containing a command with a request for sensor reports, reported periodically. (See Ref.
 * Manual 6.5.4)
 *
 * @param report_ID ID of sensor report being requested.
 * @param time_between_reports Desired time between reports.
 *
 * @return void, nothing to return
 */
//void BNO08x_queue_feature_command(BNO08x* device, uint8_t report_ID, uint32_t time_between_reports, uint32_t specific_config)
//{
//    queue_feature_command(report_ID, time_between_reports, 0); // No specific config
//}

/**
 * @brief Task responsible for SPI transactions. Executed when HINT in is asserted by BNO08x
 *
 * @return void, nothing to return
 */
void BNO08x_spi_task(void *arg)
{
    BNO08x* device = (BNO08x*) arg; // cast argument received by xTaskCreate ("this" pointer to imu object created by constructor call)
    static uint64_t prev_time = 0;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until notified by ISR

        if(device->imu_config.debug_en)
        {
            ESP_LOGI(TAG, "HINT asserted, time since last assertion: %llu", (esp_timer_get_time() - prev_time));
            prev_time = esp_timer_get_time();
        }

        if (xSemaphoreTake(device->tx_semaphore, 0) == pdTRUE) // check for packet pending to be sent, non-blocking semaphore take
            BNO08x_send_packet(device);                             // send packet
        else
            BNO08x_receive_packet(device); // receive packet

        xSemaphoreGive(device->int_asserted_semaphore); // SPI completed, give int_asserted_semaphore to notify wait_for_int()
    }
}

/**
 * @brief HINT interrupt service routine, handles falling edge of BNO08x HINT pin.
 *
 * ISR that launches SPI task to perform transaction upon assertion of BNO08x interrupt pin.
 *
 * @return void, nothing to return
 */
void IRAM_ATTR BNO08x_hint_handler(void* arg)
{
    BaseType_t xHighPriorityTaskWoken = pdFALSE;
    BNO08x* imu = (BNO08x*) arg; // cast argument received by gpio_isr_handler_add ("this" pointer to imu object
    // created by constructor call)

    gpio_intr_disable(imu->imu_config.io_int);                          // disable interrupts
    vTaskNotifyGiveFromISR(imu->spi_task_hdl, &xHighPriorityTaskWoken); // notify SPI task BNO08x is ready for
    // servicing
    portYIELD_FROM_ISR(xHighPriorityTaskWoken);                         // perform context switch if necessary
}