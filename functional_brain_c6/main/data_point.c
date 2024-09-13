#include "data_point.h"

// Turns a data point into a string to be uploaded to the SD Card
char *stringify_data_point(DataPoint *data)
{
    // Estimate buffer size
    size_t buffer_size = 1024;
    char *result = (char *)malloc(buffer_size);
    if (result == NULL)
    {
        printf("Memory allocation failed\n");
        return NULL;
    }

    // Convert time to a string
    char time_buffer[30];
    struct tm timeinfo;

    localtime_r(&data->time, &timeinfo);

    strftime(time_buffer, 30, "%Y-%m-%d %H:%M:%S", &timeinfo);

    // Create the final string
    snprintf(result, buffer_size, "%s,%d,%s,%s,%s,%s\n",
             data->command,
             data->address,
             data->name,
             time_buffer,
             data->data,
             data->units);

    return result;
}

// Function to dynamically allocate and copy a DataPoint
DataPoint *createDataPoint(DataPoint *dp)
{

    if (!dp)
        return NULL;

    DataPoint *new_dp = (DataPoint *)malloc(sizeof(DataPoint));
    if (!new_dp)
    {
        printf("Memory allocation failed\n");
        return NULL;
    }

    new_dp->address = dp->address;
    new_dp->time = dp->time;
    new_dp->name = dp->name ? strdup(dp->name) : NULL;
    new_dp->data = dp->data ? strdup(dp->data) : NULL;
    new_dp->units = dp->units ? strdup(dp->units) : NULL;
    new_dp->command = dp->command ? strdup(dp->command) : NULL;

    return new_dp;
}

// Function to free a DataPoint
void freeDataPoint(DataPoint *dp)
{
    if (dp)
    {
        free(dp->name);
        free(dp->data);
        free(dp->units);
        free(dp->command);
        free(dp);
    }
}
