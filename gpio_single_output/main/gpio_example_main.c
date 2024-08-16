/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/gpio.h"

#define GPIO_OUTPUT1_IO    1    // GPIO number to use
#define GPIO_OUTPUT2_IO    2    // GPIO number to use

void app_main(void)
{
    // Zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // Disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // Set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // Bit mask of the pins that you want to set, e.g., GPIO1
    io_conf.pin_bit_mask = (1ULL << GPIO_OUTPUT_IO);
    // Disable pull-down mode
    io_conf.pull_down_en = 0;
    // Disable pull-up mode
    io_conf.pull_up_en = 0;
    // Configure GPIO with the given settings
    gpio_config(&io_conf);

    // Set the GPIO pin to high
    gpio_set_level(GPIO_OUTPUT1_IO, 1);
    // Set the GPIO pin to high
    gpio_set_level(GPIO_OUTPUT2_IO, 1);

}
