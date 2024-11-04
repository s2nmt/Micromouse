#ifndef QUEUE_H
#define QUEUE_H

#include "Main.h"

typedef struct {
    coord items[MAX_SIZE];
    int front;
    int rear;
} Queue;

void initializeQueue(Queue* q);
bool isEmptyQueue(Queue* q);
bool isFullQueue(Queue* q);

void pushQueue(Queue* q, coord value);
void popQueue(Queue* q);
coord  peekQueue(Queue* q);
void printQueue(Queue* q);
int sizeQueue(Queue* q);
#endif