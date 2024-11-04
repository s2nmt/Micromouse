#include "queue.h"


void initializeQueue(Queue* q)
{
    q->front = -1;
    q->rear = 0;
}

bool isEmptyQueue(Queue* q) { 
    return (q->front == q->rear - 1);
}

bool isFullQueue(Queue* q) { 
    return (q->rear == MAX_SIZE); 
}

void pushQueue(Queue* q, coord value)
{
    if (isFullQueue(q)) {
        // printf("Queue is full\n");
        return;
    }
    q->items[q->rear] = value;
    q->rear++;
}

void popQueue(Queue* q)
{
    if (isEmptyQueue(q)) {
        printf("Queue is empty\n");
        return;
    }
    q->front++;
}

coord peekQueue(Queue* q)
{
    if (isEmptyQueue(q)) {
        coord empty = {-1, -1, -1};
        printf("Queue is empty\n");
        return empty;
    }
    return q->items[q->front + 1];
}

void printQueue(Queue* q) {
    if (isEmptyQueue(q)) {
        printf("Queue is empty\n");
        fprintf(stderr, "%s\n", "Queue is empty\n");
        fflush(stderr);
        return;
    }

    printf("Current Queue: ");
    fprintf(stderr, "%s\n", "Current Queue: \n");
    fflush(stderr);
    for (int i = q->front + 1; i < q->rear; i++) {
        printf("(%d, %d, %d) ", q->items[i].row, q->items[i].col, q->items[i].value);
        fprintf(stderr, "(%d, %d, %d)", q->items[i].row, q->items[i].col, q->items[i].value);
        fflush(stderr);
    }
    printf("\n");
    fprintf(stderr, "%s", "\n");
    fflush(stderr);
}

int sizeQueue(Queue* q){
    return q->rear - q->front - 1;
}

