#include "fifo.h"

int Mult(int a, int b)
{
    return a*b;
}

// Function to initialize the queue
void initializeQueue(Queue* q) {
    q->front = 0;
    q->rear = 0;
    q->near_empty = true;
}

// Function to check if the queue is empty
bool isEmpty(Queue* q) {
    return (q->front == q->rear && q->near_empty);
}

// Function to check if the queue is full
bool isFull(Queue* q) {
    return (q->front == q->rear && !q->near_empty);
}

// Function to add an element to the queue (Enqueue operation)
void push(Queue* q, void* value) {
    if (isFull(q)) {
        return;
    }

    q->items[q->rear] = value;
    q->rear = (q->rear + 1) % MAX_SIZE;
    q->near_empty = false;
}

// Function to remove an element from the queue (Dequeue operation)
void* pop(Queue* q) {
    if (isEmpty(q)) {
        return NULL;
    }

    void* value = q->items[q->front];
    q->front = (q->front + 1) % MAX_SIZE;

    if (q->front == q->rear) {
        q->near_empty = true;
    }

    return value;
}

