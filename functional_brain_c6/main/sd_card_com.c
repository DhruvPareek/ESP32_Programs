#include "sd_card_com.h"

#include <string.h>
#include <time.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define EXAMPLE_MAX_CHAR_SIZE 128

#define BUFFER_SIZE 1025
#define SEPARATOR "***"
#define SEPARATOR_SIZE 3 // Length of the separator

static const char *SD_COM_TAG = "SD_COM_TAG";
static const char *UNIQUE_TAG = "UNIQUE_IDENTIFIER";

// Upload Queued data to the SD Card
esp_err_t s_paste_data_queue_file(const char *path, Queue *q)
{
    // ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "a");
    if (f == NULL)
    {
        ESP_LOGE(SD_COM_TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    while (true)
    {
        DataPoint *temp_data_point = pop(q);

        if (temp_data_point == NULL)
        {
            break;
        }

        char *temp_data_string = stringify_data_point(temp_data_point);
        // printf(temp_data_string);

        fprintf(f, temp_data_string);
        freeDataPoint(temp_data_point);
    }

    fclose(f);
    // ESP_LOGI(SD_COM_TAG, "File written");

    return ESP_OK;
}

esp_err_t send_data_over_usb(const char *path)
{
    // ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL)
    {
        ESP_LOGE(SD_COM_TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGE(SD_COM_TAG, "Opened Files");
    }

    char buffer[BUFFER_SIZE];
    char line[EXAMPLE_MAX_CHAR_SIZE];
    size_t buffer_len = 0;

    buffer[0] = '\0'; // Initialize the buffer as an empty string

    while (fgets(line, sizeof(line), f) != NULL)
    {
        // Strip newline
        char *pos = strchr(line, '\n');
        if (pos)
        {
            *pos = '\0';
        }

        size_t line_len = strlen(line);
        size_t separator_len = buffer_len == 0 ? 0 : SEPARATOR_SIZE;

        if (buffer_len + line_len + separator_len >= BUFFER_SIZE)
        {
            ESP_LOGI(UNIQUE_TAG, "%s", buffer);
            vTaskDelay(1 / portTICK_PERIOD_MS);

            buffer[0] = '\0'; // Clear the buffer
            buffer_len = 0;
        }

        if (buffer_len > 0)
        {
            strcat(buffer, SEPARATOR);
            buffer_len += SEPARATOR_SIZE;
        }

        strcat(buffer, line);
        buffer_len += line_len;
    }

    if (buffer_len > 0)
    {
        ESP_LOGI(UNIQUE_TAG, "%s", buffer);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    fclose(f);
    return ESP_OK;
}

// Make sure solid connection before transfer data
bool is_stable_connection()
{
    static bool last_state = false;
    static uint32_t last_time = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool current_state = usb_serial_jtag_is_connected();

    if (current_state != last_state)
    {
        last_time = current_time;
        last_state = current_state;
    }

    return (current_time - last_time) > 10 ? current_state : !current_state;
}