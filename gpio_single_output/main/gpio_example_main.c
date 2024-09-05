/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"  // Include FreeRTOS header
#include "freertos/task.h"      // Include task-related functions

#define GPIO_OUTPUT1_IO    13    // GPIO number to use
#define GPIO_OUTPUT2_IO    12    // GPIO number to use
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT1_IO) | (1ULL<<GPIO_OUTPUT2_IO))


void app_main(void)
{
    // Zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // Disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // Set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // Disable pull-down mode
    io_conf.pull_down_en = 0;
    // Disable pull-up mode
    io_conf.pull_up_en = 0;
    // Configure GPIO with the given settings
    gpio_config(&io_conf);

    vTaskDelay(100);


    while(1){
        printf("loop\n");
    // Set the GPIO pin to high
    gpio_set_level(GPIO_OUTPUT1_IO, 1);
    // Set the GPIO pin to high
    gpio_set_level(GPIO_OUTPUT2_IO, 1);

    

    vTaskDelay(100);

        // Set the GPIO pin to high
    gpio_set_level(GPIO_OUTPUT1_IO, 0);

        vTaskDelay(100);

    // Set the GPIO pin to high
    gpio_set_level(GPIO_OUTPUT2_IO, 0);
            vTaskDelay(100);

    }


}
