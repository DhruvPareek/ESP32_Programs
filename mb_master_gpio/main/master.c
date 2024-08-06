/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "modbus.h"


#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
// #define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

#define ESP_INTR_FLAG_DEFAULT 0

uint8_t EDDISON_DETECTION_IO_VAL = 0; // Initial value for GPIO 11
uint8_t BATES_DETECTION_IO_VAL = 0; // Initial value for GPIO 12

uint8_t INPUT_40AMP_SWITCH_VAL = 0; // Initial value for GPIO 12
uint8_t INPUT_50AMP_SWITCH_VAL = 0; // Initial value for GPIO 1
uint8_t INPUT_60AMP_SWITCH_VAL = 0; // Initial value for GPIO 0

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
        gpio_intr_disable(gpio_num);
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int level = gpio_get_level(io_num);
            bool stable = false;
            vTaskDelay(100/ portTICK_PERIOD_MS);

            if(io_num == EDDISON_DETECTION_IO){
                stable = (EDDISON_DETECTION_IO_VAL != level);
                EDDISON_DETECTION_IO_VAL = level;
            } else if (io_num == BATES_DETECTION_IO) {
                stable = (BATES_DETECTION_IO_VAL != level);
                BATES_DETECTION_IO_VAL = level;
            }

            if(stable){
                // NEITHER PLUGGED IN
                if(EDDISON_DETECTION_IO_VAL && BATES_DETECTION_IO_VAL){
                    printf("Eddison and Bates register HIGH (NEITHER PLUGGED IN) \n");
                    gpio_set_level(EDDISON_SSR_SELECT_IO, 0); // Set GPIO 20 to high
                    gpio_set_level(BATES_SSR_SELECT_IO, 0); // Set GPIO 21 to low
                }
                //
                else if(EDDISON_DETECTION_IO_VAL && !BATES_DETECTION_IO_VAL){
                    printf("Eddison registers HIGH, Bates registers LOW (BATES PLUGGED IN)\n");
                    gpio_set_level(EDDISON_SSR_SELECT_IO, 0); // Set GPIO 20 to high
                    gpio_set_level(BATES_SSR_SELECT_IO, 1); // Set GPIO 21 to low
                }
                else if(!EDDISON_DETECTION_IO_VAL && BATES_DETECTION_IO_VAL){
                    printf("Eddison registers LOW, Bates registers HIGH (EDDISON PLUGGED IN)\n");
                    gpio_set_level(EDDISON_SSR_SELECT_IO, 1); // Set GPIO 20 to high
                    gpio_set_level(BATES_SSR_SELECT_IO, 0); // Set GPIO 21 to low
                }
                // BOTH PLUGGED IN
                else if (!EDDISON_DETECTION_IO_VAL && !BATES_DETECTION_IO_VAL){
                    printf("Eddison and Bates registers LOW (BOTH PLUGGED IN) \n");
                    gpio_set_level(EDDISON_SSR_SELECT_IO, 0); // Set GPIO 20 to high
                    gpio_set_level(BATES_SSR_SELECT_IO, 1); // Set GPIO 21 to low
                }
                master_operation_func(NULL);
            }

            gpio_intr_enable(io_num);
            
        }
    }
}

void app_main(void)
{
     gpio_config_t io_conf = {};
        
    // Configure GPIO 11 as input with pull-up and interrupts on any edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = AC_DETECTION_IN;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Configure GPIO 20 and 21 as outputs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = SSR_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    // Start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    // Install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // Hook isr handler for specific gpio pin
    gpio_isr_handler_add(EDDISON_DETECTION_IO, gpio_isr_handler, (void*) EDDISON_DETECTION_IO);
    gpio_isr_handler_add(BATES_DETECTION_IO, gpio_isr_handler, (void*) BATES_DETECTION_IO);

    gpio_isr_handler_add(INPUT_40AMP_SWITCH, gpio_isr_handler, (void*) INPUT_40AMP_SWITCH);
    gpio_isr_handler_add(INPUT_50AMP_SWITCH, gpio_isr_handler, (void*) INPUT_50AMP_SWITCH);
    gpio_isr_handler_add(INPUT_60AMP_SWITCH, gpio_isr_handler, (void*) INPUT_60AMP_SWITCH);

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());
    
    //Assuming that A0 and A1 start out at 1,1. Mosi and Miso should be set to 0&0
    gpio_set_level(BATES_SSR_SELECT_IO, 0);
    gpio_set_level(EDDISON_SSR_SELECT_IO, 0);

    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);

    int cnt = 0;
    // gpio_set_level(EDDISON_SSR_SELECT_IO, 1); // Set GPIO 20 to high
    // gpio_set_level(BATES_SSR_SELECT_IO, 1); // Set GPIO 21 to lowz

    EDDISON_DETECTION_IO_VAL = gpio_get_level(EDDISON_DETECTION_IO);
    BATES_DETECTION_IO_VAL = gpio_get_level(BATES_DETECTION_IO);

    INPUT_40AMP_SWITCH_VAL = gpio_get_level(INPUT_40AMP_SWITCH);
    INPUT_50AMP_SWITCH_VAL = gpio_get_level(INPUT_50AMP_SWITCH);
    INPUT_60AMP_SWITCH_VAL = gpio_get_level(INPUT_60AMP_SWITCH);

    // gpio_set_level(EDDISON_SSR_SELECT_IO, 1); // Set GPIO 20 to high
    // gpio_set_level(BATES_SSR_SELECT_IO, 1); // Set GPIO3 21 to low
    while(1){
        cnt++;
        printf("cnt: %d\n", cnt);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
