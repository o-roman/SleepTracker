/**
 * @file main.c
 * @author Oliver Roman
 * @brief Application main entry point
 * @version 0.1
 * @date 2024-11-04
 */

#include <stdio.h>


void main(void) {
    
    int err = -1;
    printf("Initialising board... \n");

    // Add err = board_init function
    if (err) {
        printf("Board failed to initialise with error: %d\n", err);
        return;
    }

    // Add err = bluetooth_init function
    if (err) {
        printf("Bluetooth failed to initialise with error: %d\n", err);
    }
}