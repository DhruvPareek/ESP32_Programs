#ifndef MODBUS_H
#define MODBUS_H


#include "string.h"
#include "esp_log.h"
#include "modbus_params.h"  // for modbus parameters structures
#include "mbcontroller.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define GPIO_INPUT_IO_0     18 //A0
#define GPIO_INPUT_IO_1     19 //A1
#define GPIO_OUTPUT_IO_0    10 //MOSI
#define GPIO_OUTPUT_IO_1    11 //MISO

extern QueueHandle_t gpio_evt_queue;
extern const char *TAG;
extern const uint16_t num_device_parameters;
extern const mb_parameter_descriptor_t device_parameters[];
int sum(int one, int two);

esp_err_t master_init(void);
void* master_get_param_data(const mb_parameter_descriptor_t* param_descriptor);
void master_operation_func(void *arg, int GPIO_INPUT_IO_0_VAL, int GPIO_INPUT_IO_1_VAL);


#endif  
