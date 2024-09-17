#ifndef FIFO_H
#define FIFO_H


#include <stdbool.h>
#include <stdio.h>

#define MAX_SIZE 200


// Defining the Queue structure
typedef struct {
    void* items[MAX_SIZE];
    int front;
    int rear;
    bool near_empty;
} Queue;


int Mult(int a, int b);



void initializeQueue(Queue* q);
bool isEmpty(Queue* q);
bool isFull(Queue* q);
void push(Queue* q, void* value);
void* pop(Queue* q);








#endif
