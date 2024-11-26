/**
 * @file main.c
 * @author Oliver Roman
 * @brief Application main entry point
 * @version 0.1
 * @date 2024-11-25
 */

#include <stdio.h>
#include "driver/gpio.h"

int app_main() {
    
    int err = -1;
    printf("Initialising board... \n");

    // Add err = board_init function
    if (err) {
        printf("Board failed to initialise with error: %d\n", err);
        return err;
    }

    // Add err = bluetooth_init function
    if (err) {
        printf("Bluetooth failed to initialise with error: %d\n", err);
    }

    return err;
}