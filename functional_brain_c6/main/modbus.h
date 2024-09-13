#ifndef MODBUS_H
#define MODBUS_H

#include "string.h"
#include "esp_log.h"
#include "modbus_params.h" // for modbus parameters structures
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

#include "sd_card_com.h"

#include "esp_system.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

// Input
#define EDDISON_DETECTION_IO 18
#define BATES_DETECTION_IO 19
#define INPUT_40AMP_SWITCH 0
#define INPUT_60AMP_SWITCH 1
// Output
#define EDDISON_SSR_SELECT_IO 13
#define BATES_SSR_SELECT_IO 12

// Defined here to be accessible in both Master.c to obtain vals and Modbus.c to do stuff with vals
extern uint8_t INPUT_40AMP_SWITCH_VAL; // Initial value for GPIO 8
extern uint8_t INPUT_60AMP_SWITCH_VAL; // Initial value for GPIO 0

// The different things we will ask modbus_master_func to do
typedef enum
{
    WRITE_DATA = 0,
    READ_MEANWELL,
    READ_JDB,

} OperationType;

extern const char *TAG;
extern const uint16_t num_device_parameters;
extern const mb_parameter_descriptor_t device_parameters[];
int sum(int one, int two);

esp_err_t master_init(void);
void *master_get_param_data(const mb_parameter_descriptor_t *param_descriptor);
void master_operation_func(void *arg, int EDDISON_VAL, int BATES_VAL, OperationType op_type, Queue *queue);

void get_time_string(char *time_str, size_t max_len);
void set_time_manually(int year, int month, int day, int hour, int minute, int second);
void set_value(int edd_val, int bates_val, uint16_t *val);
void set_ssrs(int edd_val, int bates_val);

#endif
