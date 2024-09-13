#ifndef DATAPOINT_H
#define DATAPOINT_H

#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

typedef struct
{
    int address;
    char *name;
    time_t time;
    char *data;
    char *units;
    char *command;
} DataPoint;

char *stringify_data_point(DataPoint *data);
DataPoint *createDataPoint(DataPoint *dp);
void freeDataPoint(DataPoint *dp);

#endif
