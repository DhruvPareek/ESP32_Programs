#ifndef SD_CARD_COM_H
#define SD_CARD_COM_H

#include "sdmmc_cmd.h"
#include "fifo.h"
#include "data_point.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"


char *stringify_data_point(DataPoint *data);

esp_err_t s_paste_data_queue_file(const char *path, Queue *q);
esp_err_t send_data_over_usb(const char *path);

bool is_stable_connection();

int printConstant(char *labal);

#endif
