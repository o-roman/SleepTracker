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
#include "soc/clk_tree_defs.h"
#include "esp_mac.h"

#define DATA_BUFF 8
#define TMP117_ADDR 0x48
#define MC3479 0x6C
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
        .i2c_port = I2C_NUM_0,
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

    //ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &tmp117_config, &tmp117_handle));
    //printf("tmp PASSED\n");

    // TODO Write the Pointer Address first and then try read. #ADDR #PTRADDR #ADDR #READ
    //ESP_ERROR_CHECK(i2c_master_receive(tmp117_handle, test_buff, DATA_BUFF, -1));

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &max32664_config, &max32664_handle));
    printf("max PASSED\n");
    
    uint8_t test_buff[DATA_BUFF] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t read_status_byte = 0xFF;
    uint8_t buff[3];
    uint8_t max32664_mode_write[3] = {0x10, 0x00, 0x01};
    uint8_t max32664_interrupt_threshold[3] = {0x10, 0x01, 0x0F};
    uint8_t max30101_mode_on[3] = {0x44, 0x03, 0x01};
    uint8_t max32644_hr_algo[3] = {0x52, 0x02, 0x01};
    uint8_t max30101_agc_mode_off[3] = {0x52, 0x00, 0x00};
    uint8_t max30101_led1_mode[3] = {0x40, 0x03, 0x0C, 0x7F};
    uint8_t max30101_led2_mode[3] = {0x40, 0x03, 0x0D, 0x7F};
    uint8_t max30101_num_sample_FIFO[2] = {0x12, 0x00};
    uint8_t max30101_read_FIFO[2] = {0x12, 0x01};
    uint8_t max32644_status[2] = {0x00, 0x00};

    // Startup for the MAX32664
    gpio_set_level(GPIO_RST, 0);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_MFIO, 1);
    vTaskDelay(5/ portTICK_PERIOD_MS);
    gpio_set_level(GPIO_RST, 1);

    vTaskDelay(1500 / portTICK_PERIOD_MS);

    // Running
    // Turn on sensors
    ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, max32664_mode_write, (uint8_t) sizeof(max32664_mode_write), -1));
    vTaskDelay(5/ portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, &read_status_byte, (uint8_t) sizeof(read_status_byte), -1));
    
    ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, max32664_interrupt_threshold, (uint8_t) sizeof(max32664_interrupt_threshold), -1));
    vTaskDelay(5/ portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, &read_status_byte, (uint8_t) sizeof(read_status_byte), -1));

    ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, max30101_mode_on, (uint8_t) sizeof(max30101_mode_on), -1));
    vTaskDelay(5/ portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, &read_status_byte, (uint8_t) sizeof(read_status_byte), -1));

    ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, max32644_hr_algo, (uint8_t) sizeof(max32644_hr_algo), -1));
    vTaskDelay(5/ portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, &read_status_byte, (uint8_t) sizeof(read_status_byte), -1));

    ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, max30101_agc_mode_off, (uint8_t) sizeof(max30101_agc_mode_off), -1));
    vTaskDelay(5/ portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, &read_status_byte, (uint8_t) sizeof(read_status_byte), -1));

    // Wait 100 ms to allow sensors to turn on
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, max30101_led1_mode, (uint8_t) sizeof(max30101_led1_mode), -1));
    vTaskDelay(5/ portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, &read_status_byte, (uint8_t) sizeof(read_status_byte), -1));

    ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, max30101_led2_mode, (uint8_t) sizeof(max30101_led2_mode), -1));
    vTaskDelay(5/ portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, &read_status_byte, (uint8_t) sizeof(read_status_byte), -1));

    printf("Result of sensor hub mode check: 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", test_buff[0], test_buff[1], test_buff[2], test_buff[3], test_buff[4], \
                     test_buff[5],  test_buff[6],  test_buff[7]);

    // Clearing test_buff
    memset(test_buff, 0, sizeof test_buff);

    while (1) {
        printf("Bytes contained are: ");
        for(int i = 0; i < DATA_BUFF; i++) {
            printf("0x%x ", test_buff[i]);
        }
        printf("\n");

        ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, max32644_status, (uint8_t) sizeof(max32644_status), -1));
        vTaskDelay(5/ portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, &read_status_byte, (uint8_t) sizeof(read_status_byte), -1));

        // Send read request for sensors
        ESP_ERROR_CHECK(i2c_master_transmit(max32664_handle, buff, (uint8_t) sizeof(buff), -1));
        
        vTaskDelay(5 / portTICK_PERIOD_MS);

        // Actually read sensor data
        ESP_ERROR_CHECK(i2c_master_receive(max32664_handle, test_buff, DATA_BUFF, -1));

        vTaskDelay(1000/ portTICK_PERIOD_MS);
        
    }
}