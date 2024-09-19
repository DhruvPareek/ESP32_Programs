/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "modbus.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define AC_DETECTION_IN ((1ULL << EDDISON_DETECTION_IO) | (1ULL << BATES_DETECTION_IO))

#define SSR_PIN_SEL ((1ULL << EDDISON_SSR_SELECT_IO) | (1ULL << BATES_SSR_SELECT_IO))

#define TRI_SWITCH_IN ((1ULL << INPUT_40AMP_SWITCH) | (1ULL << INPUT_60AMP_SWITCH))

#define ESP_INTR_FLAG_DEFAULT 0
#define DEBOUNCE_TIME_MS 10 // 500 milliseconds

// SD Card Declarations
#define MOUNT_POINT "/sdcard"
#define EXAMPLE_MAX_CHAR_SIZE 64

#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 7
#define PIN_NUM_CLK 6
#define PIN_NUM_CS 21

uint8_t EDDISON_DETECTION_IO_VAL = 1; // Initial value for GPIO 10
uint8_t BATES_DETECTION_IO_VAL = 1;   // Initial value for GPIO 11

uint8_t INPUT_40AMP_SWITCH_VAL = 0; // Initial value for GPIO 12
uint8_t INPUT_60AMP_SWITCH_VAL = 0; // Initial value for GPIO 0

static QueueHandle_t gpio_evt_queue = NULL;

static Queue q;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    gpio_intr_disable(gpio_num); // Disable interrupt for the pin
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static bool debounce(uint32_t io_num, uint8_t *last_val)
{
    uint8_t stable_val = gpio_get_level(io_num);
    // TickType_t og_start_tick = xTaskGetTickCount();
    TickType_t start_tick = xTaskGetTickCount();
    printf("Oscillating\n");
    // int num_oscillations = 0;
    // bool succesful = true;
    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(DEBOUNCE_TIME_MS))
    {
        uint8_t current_val = gpio_get_level(io_num);
        if (current_val != stable_val)
        {
            stable_val = current_val;
            vTaskDelay(1);                    // Neccessary to free up other tasks
            start_tick = xTaskGetTickCount(); // Restart debounce timer
        }
        // Exit if its been X seconds since we've started debouncing. Important, but can lead to bug where it is plugged/unplugged but exits as the other, and doesnt re-trigger
        // if(xTaskGetTickCount() - og_start_tick > pdMS_TO_TICKS(DEBOUNCE_TIME_MS * 5)){
        //     succesful = false;
        //     break;
        // }
    }
    printf("Finished!\n");

    if (stable_val != *last_val)
    {
        *last_val = stable_val;
        return true;
    }
    return false;
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            bool stable = false;
            if (io_num == INPUT_40AMP_SWITCH)
            {
                stable = debounce(io_num, &INPUT_40AMP_SWITCH_VAL);
            }
            if (io_num == INPUT_60AMP_SWITCH)
            {
                stable = debounce(io_num, &INPUT_60AMP_SWITCH_VAL);
            }
            if (io_num == EDDISON_DETECTION_IO)
            {
                stable = debounce(io_num, &EDDISON_DETECTION_IO_VAL);
            }
            else if (io_num == BATES_DETECTION_IO)
            {
                stable = debounce(io_num, &BATES_DETECTION_IO_VAL);
            }
            gpio_intr_enable(io_num);
            if (stable)
            {
                master_operation_func(NULL, EDDISON_DETECTION_IO_VAL, BATES_DETECTION_IO_VAL, WRITE_DATA, &q);
            }
        }
    }
}

void app_main(void)
{
    initializeQueue(&q);
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    bool card_initialized = true;

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        card_initialized = false;
        // return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files.

    // Format FATFS
#ifdef CONFIG_EXAMPLE_FORMAT_SD_CARD
    ret = esp_vfs_fat_sdcard_format(mount_point, card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to format FATFS (%s)", esp_err_to_name(ret));
        return;
    }

    if (stat(file_foo, &st) == 0)
    {
        ESP_LOGI(TAG, "file still exists");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "file doesnt exist, format done");
    }
#endif // CONFIG_EXAMPLE_FORMAT_SD_CARD

    // GPIO Stuff

    gpio_config_t io_conf = {};

    // Configure as input with pull-up and interrupts on any edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = AC_DETECTION_IN;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Configure as input with pull-up and interrupts on any edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = TRI_SWITCH_IN;
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

    gpio_set_level(INPUT_60AMP_SWITCH, 0);

    // Hook isr handler for specific gpio pin
    gpio_isr_handler_add(EDDISON_DETECTION_IO, gpio_isr_handler, (void *)EDDISON_DETECTION_IO);
    gpio_isr_handler_add(BATES_DETECTION_IO, gpio_isr_handler, (void *)BATES_DETECTION_IO);

    gpio_isr_handler_add(INPUT_40AMP_SWITCH, gpio_isr_handler, (void *)INPUT_40AMP_SWITCH);
    // gpio_isr_handler_add(INPUT_50AMP_SWITCH, gpio_isr_handler, (void*) INPUT_50AMP_SWITCH);
    gpio_isr_handler_add(INPUT_60AMP_SWITCH, gpio_isr_handler, (void *)INPUT_60AMP_SWITCH);

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // Assuming that A0 and A1 start out at 0,0. Mosi and Miso should be set to 0&1
    gpio_set_level(EDDISON_SSR_SELECT_IO, 1);
    gpio_set_level(BATES_SSR_SELECT_IO, 0);

    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);

    int cnt = 0;

    EDDISON_DETECTION_IO_VAL = gpio_get_level(EDDISON_DETECTION_IO);
    BATES_DETECTION_IO_VAL = gpio_get_level(BATES_DETECTION_IO);
    INPUT_40AMP_SWITCH_VAL = gpio_get_level(INPUT_40AMP_SWITCH);
    INPUT_60AMP_SWITCH_VAL = gpio_get_level(INPUT_60AMP_SWITCH);
    // TODO - save file under
    const char *file_data = MOUNT_POINT "/data1.txt";

    setenv("TZ", "PST8PDT,M3.2.0,M11.1.0", 1);
    tzset();

    // Set time manually (Year, Month, Day, Hour, Minute, Second)
    set_time_manually(2024, 8, 7, 10, 30, 0);

    ESP_LOGI(TAG, "Time set manually");

    // Get and print current time
    // char time_string[64];
    // get_time_string(time_string, sizeof(time_string));
    // ESP_LOGI(TAG, "Current time: %s", time_string);

    master_operation_func(NULL, EDDISON_DETECTION_IO_VAL, BATES_DETECTION_IO_VAL, READ_MEANWELL, &q); // Set og Current

    while (1)
    {
        cnt++;
        printf("cnt: %d\n", cnt);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (cnt % 30 == 0)
        {
            master_operation_func(NULL, EDDISON_DETECTION_IO_VAL, BATES_DETECTION_IO_VAL, READ_MEANWELL, &q);
            cnt = 1;
        }
        
        int edd_val = gpio_get_level(EDDISON_DETECTION_IO);
        int bates_val = gpio_get_level(BATES_DETECTION_IO);

        //SSR's must be set in the master.c file and not modbus.c because the gpio pins high/low are not able to be changed in modbus.c
        if (edd_val && bates_val)
        {
            gpio_set_level(EDDISON_SSR_SELECT_IO, 0);
            gpio_set_level(BATES_SSR_SELECT_IO, 0);
            printf("Eddison and Bates register HIGH (NEITHER PLUGGED IN) \n");
        }
        //bates plugged in
        else if (edd_val && !bates_val)
        {
            gpio_set_level(EDDISON_SSR_SELECT_IO, 0);
            gpio_set_level(BATES_SSR_SELECT_IO, 1);
            printf("Eddison registers HIGH, Bates registers LOW (BATES PLUGGED IN)\n");
        }//edisson plugged in
        else if (!edd_val && bates_val)
        {
            gpio_set_level(EDDISON_SSR_SELECT_IO, 1);
            gpio_set_level(BATES_SSR_SELECT_IO, 0);
            printf("Eddison registers LOW, Bates registers HIGH (EDDISON PLUGGED IN)\n");
        }
        // BOTH PLUGGED IN
        else if (!edd_val && !bates_val)
        {
            gpio_set_level(EDDISON_SSR_SELECT_IO, 0);
            gpio_set_level(BATES_SSR_SELECT_IO, 1);
            printf("Eddison and Bates registers LOW (BOTH PLUGGED IN) \n");
        }
            
        if (card_initialized)
        {
            ret = s_paste_data_queue_file(file_data, &q);
            if (ret != ESP_OK)
            {
                printf("\nFailed Upload to SD Card!\n");
            } 
        }
        // printf("\n\n%s\n\n", stringify_data_point(q.items[0]));

        if (is_stable_connection() && card_initialized)
        {
            ESP_LOGI(TAG, "\n\nGOING DOWN\n\n");
            send_data_over_usb(file_data);
        }
    }
    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");

    // deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
}
