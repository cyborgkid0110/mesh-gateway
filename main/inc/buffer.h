#ifndef BUFFER_H
#define BUFFER_H

#include <stdint.h>
#include <string.h>

#define MAX_ARG_SIZE 50
#define BUFFER_MAX 10

typedef struct ipac_uart_cmd_buffer {
    uint8_t opcode;                     /* Message opcode */
    uint8_t arg[MAX_ARG_SIZE];                    /* Message arguments */
    uint8_t len;                        /* Length of the arguments */
    void (*handler)(void*, uint8_t);    /* Function to handle the command*/
} ipac_uart_cmd_buffer_t;

typedef struct ipac_uart_cmd_queue {
    ipac_uart_cmd_buffer_t items[BUFFER_MAX];
    int8_t front;
    int8_t rear;
    uint8_t size;
} ipac_uart_cmd_queue_t;

// Initialize the queue
void ipac_uart_cmd_queue_init(ipac_uart_cmd_queue_t *q) {
    q->front = -1;
    q->rear = -1;
    q->size = 0;
}

// Check if queue is empty
uint8_t ipac_uart_cmd_queue_isEmpty(ipac_uart_cmd_queue_t *q) {
    return q->size == 0;
}

// Check if queue is full
uint8_t ipac_uart_cmd_queue_isFull(ipac_uart_cmd_queue_t *q) {
    return q->size == BUFFER_MAX;
}

// Add element to queue
uint8_t ipac_uart_cmd_queue_enqueue(ipac_uart_cmd_queue_t *q, ipac_uart_cmd_buffer_t* buf) {
    if (ipac_uart_cmd_queue_isFull(q)) {
        return 1;
    }
    
    if (ipac_uart_cmd_queue_isEmpty(q)) {
        q->front = 0;
    }
    
    q->rear = (q->rear + 1) % BUFFER_MAX;
    q->items[q->rear].opcode = buf->opcode; // Copy the entire struct
    memcpy(q->items[q->rear].arg, buf->arg, MAX_ARG_SIZE);
    q->items[q->rear].len = buf->len;
    q->items[q->rear].handler = buf->handler;
    q->size++;

    return 0;
}

// Remove element from queue
uint8_t ipac_uart_cmd_queue_dequeue(ipac_uart_cmd_queue_t *q, ipac_uart_cmd_buffer_t* buf) {
    ipac_uart_cmd_buffer_t value = {0}; // Initialize with zeros
    if (ipac_uart_cmd_queue_isEmpty(q)) {
        return 1;
    }
    
    // value = q->items[q->front];
    buf->opcode = q->items[q->front].opcode;
    memcpy(buf->arg, q->items[q->front].arg, MAX_ARG_SIZE);
    buf->len = q->items[q->front].len;
    buf->handler = q->items[q->front].handler;
    q->front = (q->front + 1) % BUFFER_MAX;
    q->size--;
    
    if (ipac_uart_cmd_queue_isEmpty(q)) {
        q->front = -1;
        q->rear = -1;
    }

    return 0;
}

#endif