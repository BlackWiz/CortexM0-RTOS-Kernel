/** @file uart.c
 *
 * @brief UART driver implementation for STM32G0 with Ring Buffers.
 *
 * Provides non-blocking interrupt-driven UART TX/RX.
 */

#include "uart.h"
#include <stddef.h>

/* Register bit position constants */
#define RCC_APBENR1_USART2_BIT 17u
#define RCC_IOPENR_GPIOA_BIT 0u
#define GPIO_MODER_AF_MODE 0x2u
#define GPIO_AFR_AF1 0x1u
#define USART_CR1_UE_BIT 0u
#define USART_CR1_TE_BIT 3u
#define USART_CR1_RE_BIT 2u
#define USART_CR1_TXEIE_BIT 7u
#define USART_CR1_RXNEIE_BIT 5u
#define USART_ISR_TXE_BIT 7u
#define USART_ISR_RXNE_BIT 5u

/* Pin configuration constants */
#define PA2_PIN_NUM 2u
#define PA3_PIN_NUM 3u
#define BITS_PER_PIN 2u
#define PA2_AFR_SHIFT 8u
#define PA3_AFR_SHIFT 12u

/* Baud rate calculation for 16MHz clock, 115200 baud */
#define BAUD_RATE_115200_AT_16MHZ 139u

/* Hardware Registers */
volatile uint32_t *USART_CR1 = (uint32_t *)0x40004400;
volatile uint32_t *USART_BRR = (uint32_t *)0x4000440C;
volatile uint32_t *USART_ISR = (uint32_t *)0x4000441C;
volatile uint32_t *USART_ICR = (uint32_t *)0x40004420;
volatile uint32_t *USART_RDR = (uint32_t *)0x40004424;
volatile uint32_t *USART_TDR = (uint32_t *)0x40004428;

volatile uint32_t *RCC_IOPENR = (uint32_t *)0x40021034;
volatile uint32_t *RCC_APBENR1 = (uint32_t *)0x4002103C;

volatile uint32_t *GPIOx_MODER = (uint32_t *)0x50000000;
volatile uint32_t *GPIOx_AFRL = (uint32_t *)0x50000020;

#define NVIC_ISER0 ((volatile uint32_t *)0xE000E100)
#define USART2_IRQn 28u

/* Ring Buffers */
static uart_ring_buffer_t tx_buffer;
static uart_ring_buffer_t rx_buffer;

/* Helper functions */
static inline void nvic_enable_irq(uint32_t irq_n) {
  NVIC_ISER0[irq_n >> 5] = (1u << (irq_n & 0x1F));
}

int32_t uart_init(void) {
  /* Enable peripheral clocks */
  *RCC_APBENR1 |= (1u << RCC_APBENR1_USART2_BIT);
  *RCC_IOPENR |= (1u << RCC_IOPENR_GPIOA_BIT);

  /* Configure PA2 (TX) and PA3 (RX) as alternate function mode */
  *GPIOx_MODER &= ~(0x3u << (BITS_PER_PIN * PA2_PIN_NUM));
  *GPIOx_MODER &= ~(0x3u << (BITS_PER_PIN * PA3_PIN_NUM));
  *GPIOx_MODER |= (GPIO_MODER_AF_MODE << (BITS_PER_PIN * PA2_PIN_NUM));
  *GPIOx_MODER |= (GPIO_MODER_AF_MODE << (BITS_PER_PIN * PA3_PIN_NUM));

  /* Set alternate function AF1 for USART2 */
  *GPIOx_AFRL &= ~(0xFu << PA2_AFR_SHIFT);
  *GPIOx_AFRL |= (GPIO_AFR_AF1 << PA2_AFR_SHIFT);
  *GPIOx_AFRL &= ~(0xFu << PA3_AFR_SHIFT);
  *GPIOx_AFRL |= (GPIO_AFR_AF1 << PA3_AFR_SHIFT);

  /* Configure baud rate */
  *USART_BRR = BAUD_RATE_115200_AT_16MHZ;

  /* Initialize Ring Buffers */
  tx_buffer.head = 0;
  tx_buffer.tail = 0;
  rx_buffer.head = 0;
  rx_buffer.tail = 0;

  /* Enable USART, TX, RX, and RXNE Interrupt */
  *USART_CR1 |= ((1u << USART_CR1_UE_BIT) | (1u << USART_CR1_TE_BIT) |
                 (1u << USART_CR1_RE_BIT) | (1u << USART_CR1_RXNEIE_BIT));

  /* Enable NVIC Interrupt */
  nvic_enable_irq(USART2_IRQn);

  return 0;
}

void uart_write(uint8_t data) {
  uint16_t next_head = (tx_buffer.head + 1) & UART_BUFFER_MASK;

  /* Wait if buffer is full (simple spinlock, ideally should yield) */
  while (next_head == tx_buffer.tail)
    ;

  tx_buffer.buffer[tx_buffer.head] = data;
  tx_buffer.head = next_head;

  /* Enable TXE interrupt */
  *USART_CR1 |= (1u << USART_CR1_TXEIE_BIT);
}

void uart_print(const char *str) {
  while (*str) {
    uart_write((uint8_t)*str++);
  }
}

bool uart_read(uint8_t *data) {
  if (rx_buffer.head == rx_buffer.tail) {
    return false;
  }

  *data = rx_buffer.buffer[rx_buffer.tail];
  rx_buffer.tail = (rx_buffer.tail + 1) & UART_BUFFER_MASK;
  return true;
}

uint16_t uart_available(void) {
  return (rx_buffer.head - rx_buffer.tail) & UART_BUFFER_MASK;
}

void USART2_IRQHandler(void) {
  /* Handle RXNE (Read Data Register Not Empty) */
  if (*USART_ISR & (1u << USART_ISR_RXNE_BIT)) {
    uint8_t data = (uint8_t)(*USART_RDR);
    uint16_t next_head = (rx_buffer.head + 1) & UART_BUFFER_MASK;

    if (next_head != rx_buffer.tail) {
      rx_buffer.buffer[rx_buffer.head] = data;
      rx_buffer.head = next_head;
    }
    /* Else: Buffer overflow, drop data */
  }

  /* Handle TXE (Transmit Data Register Empty) */
  if ((*USART_ISR & (1u << USART_ISR_TXE_BIT)) &&
      (*USART_CR1 & (1u << USART_CR1_TXEIE_BIT))) {

    if (tx_buffer.head != tx_buffer.tail) {
      *USART_TDR = tx_buffer.buffer[tx_buffer.tail];
      tx_buffer.tail = (tx_buffer.tail + 1) & UART_BUFFER_MASK;
    } else {
      /* Buffer empty, disable TXE interrupt */
      *USART_CR1 &= ~(1u << USART_CR1_TXEIE_BIT);
    }
  }
}
