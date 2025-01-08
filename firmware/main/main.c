/**
 * @file main.c
 * @author Oliver Roman
 * @brief Application main entry point
 * @version 0.1
 * @date 2024-11-25
 */

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_mac.h"

#define DATA_BUFF 5
#define TMP117_ADDR 0x48
#define MAX32664_ADDR 0x55
#define GPIO_RST GPIO_NUM_7
#define GPIO_MFIO GPIO_NUM_21

/**
 * Function that initialises all needed gpio ports 
 */
void gpio_init() {

    // Set default level high to NOT reset
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_RST, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_RST, 1));

    ESP_ERROR_CHECK(gpio_set_direction(GPIO_MFIO, GPIO_MODE_INPUT_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_MFIO, 0));
}


/**
 * Main application loop
 */
void app_main() {
    
    printf("Initialising board... \n");

    gpio_init();

    i2c_master_bus_config_t master_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = GPIO_NUM_20,
        .sda_io_num = GPIO_NUM_19,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&master_config, &bus_handle));
    printf("i2c_new_master_bus PASSED\n");
    
    i2c_device_config_t tmp117_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TMP117_ADDR,
        // Check this speed to make sure its correct
        .scl_speed_hz = 100000,
    };

    i2c_device_config_t max32664_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX32664_ADDR,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t tmp117_handle;
    i2c_master_dev_handle_t max32664_handle;

    /*ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &tmp117_config, &tmp117_handle));
    printf("tmp PASSED\n");*/
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &max32664_config, &max32664_handle));
    printf("max PASSED\n");
    
    uint8_t test_buff[DATA_BUFF] = {0, 0, 0, 0, 0};
    uint8_t buff[3] = {0x44, 0x03, 0x01};
    uint8_t output_mode[2] = {0x10, 0x00};

    // TODO Write the Pointer Address first and then try read. #ADDR #PTRADDR #ADDR #READ
    //ESP_ERROR_CHECK(i2c_master_receive(tmp117_handle, test_buff, DATA_BUFF, -1));

    // Startup for the MAX32664
    gpio_set_level(GPIO_RST, 0);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_MFIO, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_RST, 1);

    vTaskDelay(2000 / portTICK_PERIOD_MS);


    // Running
    // Turn on sensors
    ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, output_mode, (uint8_t) sizeof(output_mode), -1));

    ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, buff, (uint8_t) sizeof(buff), -1));
    printf("MAX32664 Setup completed\n");
    uint8_t test_byte = 16;
    printf("Testing hex: 0x%x\n", test_byte);

    // Wait 60 ms to allow sensors to turn on
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Populate Status Check Message
    buff[0] = 0x45;
    buff[1] = 0x03;
    buff[2] = 0x00;

    printf("CHECK: 0x%x,0x%x,0x%x,0x%x,0x%x\n", test_buff[0], test_buff[1], test_buff[2], test_buff[3], test_buff[4]);

    ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, buff, (uint8_t) sizeof(buff), -1));

    // Check Buffer
    ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, test_buff, DATA_BUFF, -1));
    
    printf("Result of sensor mode check: 0x%x,0x%x,0x%x,0x%x,0x%x\n", test_buff[0], test_buff[1], test_buff[2], test_buff[3], test_buff[4]);

    // Clearing test_buff & Populate Sensor Hub Status Check Message (same thing)
    memset(test_buff, 0, sizeof test_buff);
    printf("CHECK: 0x%x,0x%x,0x%x,0x%x,0x%x\n", test_buff[0], test_buff[1], test_buff[2], test_buff[3], test_buff[4]);

    // Send check MAX settings message
    ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, buff, (uint8_t) sizeof(buff), -1));

    // Check Buffer
    ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, test_buff, DATA_BUFF, -1));

    printf("Result of sensor hub mode check: 0x%x,0x%x,0x%x,0x%x,0x%x\n", test_buff[0], test_buff[1], test_buff[2], test_buff[3], test_buff[4]);

    // Clearing test_buff
    memset(test_buff, 0, sizeof test_buff);

    // Populate Read Message
    buff[0] = 0x12;
    buff[1] = 0x01;
    buff[2] = 0x00;
    buff[3] = 0x00;

    while (1) {
        printf("Hello n\n");
        
        for(int i = 0; i < DATA_BUFF; i++) {
            printf("Value contained at position %u is: 0x%x\n", i, test_buff[i]);
        }

        // Send read request for sensors
        ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, buff, (uint8_t) sizeof(buff), -1));

        vTaskDelay(10 / portTICK_PERIOD_MS);

        // Actually read sensor data
        ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, test_buff, DATA_BUFF, -1));

        vTaskDelay(10/ portTICK_PERIOD_MS);
    }
}