#include "stack.h"

void initializeStack(Stack *stack) {
    stack->top = -1;  
}

bool isEmptyStack(Stack *stack) {
    return stack->top == -1;  
}

bool isFullStack(Stack *stack) {
    return stack->top == MAX_SIZE - 1;  
}

void pushStack(Stack *stack, coord value) {
    if (isFullStack(stack)) {
        printf("Stack Overflow\n");
        return;
    }
    stack->arr[++stack->top] = value;
}

coord popStack(Stack *stack) {
    if (isEmptyStack(stack)) {
        printf("Stack Underflow\n");
        coord error = {-1, -1, -1}; // or any other error indication
        return error;
    }
    return stack->arr[stack->top--];
}

coord peekStack(Stack *stack) {
    if (isEmptyStack(stack)) {
        coord error = {-1, -1, -1}; // or any other error indication
        return error;
    }
    return stack->arr[stack->top];
}
int sizeStack(Stack *stack){
    return stack->top + 1;
}
void printStack(Stack *stack) {
    if (isEmptyStack(stack)) {
        printf("Stack is empty\n");
        fprintf(stderr, "%s\n", "Stack is empty\n");
        fflush(stderr);
        return;
    }
    fprintf(stderr, "%s\n", "Current Stack: \n");
    fflush(stderr);
    printf("Current Stack: ");
    for (int i = 0; i < stack->top + 1; i++) {
        printf("(%d, %d, %d) ", stack->arr[i].row, stack->arr[i].col, stack->arr[i].value);
        fprintf(stderr, "(%d, %d, %d) ", stack->arr[i].row, stack->arr[i].col, stack->arr[i].value);
        fflush(stderr);
    }
    printf("\n");
    fprintf(stderr, "%s", "\n");
    fflush(stderr);
}
