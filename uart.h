/** @file uart.h
 *
 * @brief UART driver interface for STM32G0 interrupt-driven communication.
 *
 * Uses Ring Buffers for efficient, non-blocking TX/RX.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2025 Your Name. All rights reserved.
 */

#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stdint.h>


/* Buffer size for UART Ring Buffers - Power of 2 for efficiency */
#define UART_BUFFER_SIZE 256u
#define UART_BUFFER_MASK (UART_BUFFER_SIZE - 1u)

/* UART Ring Buffer Structure */
typedef struct {
  volatile uint8_t buffer[UART_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
} uart_ring_buffer_t;

/* Public API functions */
int32_t uart_init(void);
void uart_write(uint8_t data);
void uart_print(const char *str);
bool uart_read(uint8_t *data);
uint16_t uart_available(void);

#endif /* UART_H */
