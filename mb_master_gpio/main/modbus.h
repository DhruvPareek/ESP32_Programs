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

#define INPUT_40AMP_SWITCH  8 //GPIO 12
#define INPUT_50AMP_SWITCH  1 //GPIO 12
#define INPUT_60AMP_SWITCH  0 //GPIO 12

#define EDDISON_DETECTION_IO 10
#define BATES_DETECTION_IO 11
#define AC_DETECTION_IN ((1ULL << EDDISON_DETECTION_IO) | (1ULL << BATES_DETECTION_IO))

#define EDDISON_SSR_SELECT_IO 20
#define BATES_SSR_SELECT_IO 21
#define SSR_PIN_SEL  ((1ULL << EDDISON_SSR_SELECT_IO) | (1ULL << BATES_SSR_SELECT_IO))

extern uint8_t EDDISON_DETECTION_IO_VAL; // Initial value for GPIO 11
extern uint8_t BATES_DETECTION_IO_VAL; // Initial value for GPIO 12

extern uint8_t INPUT_40AMP_SWITCH_VAL; // Initial value for GPIO 12
extern uint8_t INPUT_50AMP_SWITCH_VAL; // Initial value for GPIO 1
extern uint8_t INPUT_60AMP_SWITCH_VAL; // Initial value for GPIO 0

extern QueueHandle_t gpio_evt_queue;
extern const char *TAG;
extern const uint16_t num_device_parameters;
extern const mb_parameter_descriptor_t device_parameters[];
int sum(int one, int two);

esp_err_t master_init(void);
void* master_get_param_data(const mb_parameter_descriptor_t* param_descriptor);
void master_operation_func(void *arg);


#endif  
