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
        return;
    }
    q->items[q->rear] = value;
    q->rear++;
}

void popQueue(Queue* q)
{
    if (isEmptyQueue(q)) {
        return;
    }
    for (int i = q->front + 1; i < q->rear - 1; i++) {
        q->items[i] = q->items[i + 1];
    }
    q->rear--;
    if (q->rear == 0) {
        q->front = -1;
        q->rear = 0;
    }
}
coord peekQueue(Queue* q)
{
    if (isEmptyQueue(q)) {
        coord empty = {-1, -1, -1};
        return empty;
    }
    return q->items[q->front + 1];
}


int sizeQueue(Queue* q){
    return q->rear - q->front - 1;
}

