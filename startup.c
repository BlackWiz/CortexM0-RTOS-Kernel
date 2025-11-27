/** @file startup.c
 * @brief Startup code and vector table
 */

#include <stdint.h>

extern uint32_t _sidata;
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _top_of_stack;

extern int main(void);
void Reset_Handler(void);
void Default_Handler(void);

extern void SysTick_Handler(void);
extern void PendSV_Handler(void);
extern void USART2_IRQHandler(void);

void Reset_Handler(void) {
  uint32_t *src, *dst;

  src = &_sidata;
  dst = &_sdata;
  while (dst < &_edata) {
    *dst++ = *src++;
  }

  dst = &_sbss;
  while (dst < &_ebss) {
    *dst++ = 0;
  }

  main();

  while (1)
    ;
}

__attribute__((section(".isr_vector"))) const uint32_t g_pfnVectors[] = {
    (uint32_t)&_top_of_stack,
    (uint32_t)&Reset_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    (uint32_t)&Default_Handler,
    0,
    0,
    (uint32_t)&PendSV_Handler,
    (uint32_t)&SysTick_Handler,

    (uint32_t)&Default_Handler,   /* IRQ0 */
    (uint32_t)&Default_Handler,   /* IRQ1 */
    (uint32_t)&Default_Handler,   /* IRQ2 */
    (uint32_t)&Default_Handler,   /* IRQ3 */
    (uint32_t)&Default_Handler,   /* IRQ4 */
    (uint32_t)&Default_Handler,   /* IRQ5 */
    (uint32_t)&Default_Handler,   /* IRQ6 */
    (uint32_t)&Default_Handler,   /* IRQ7 */
    (uint32_t)&Default_Handler,   /* IRQ8 */
    (uint32_t)&Default_Handler,   /* IRQ9 */
    (uint32_t)&Default_Handler,   /* IRQ10 */
    (uint32_t)&Default_Handler,   /* IRQ11 */
    (uint32_t)&Default_Handler,   /* IRQ12 */
    (uint32_t)&Default_Handler,   /* IRQ13 */
    (uint32_t)&Default_Handler,   /* IRQ14 */
    (uint32_t)&Default_Handler,   /* IRQ15 */
    (uint32_t)&Default_Handler,   /* IRQ16 */
    (uint32_t)&Default_Handler,   /* IRQ17 */
    (uint32_t)&Default_Handler,   /* IRQ18 */
    (uint32_t)&Default_Handler,   /* IRQ19 */
    (uint32_t)&Default_Handler,   /* IRQ20 */
    (uint32_t)&Default_Handler,   /* IRQ21 */
    (uint32_t)&Default_Handler,   /* IRQ22 */
    (uint32_t)&Default_Handler,   /* IRQ23 */
    (uint32_t)&Default_Handler,   /* IRQ24 */
    (uint32_t)&Default_Handler,   /* IRQ25 */
    (uint32_t)&Default_Handler,   /* IRQ26 */
    (uint32_t)&Default_Handler,   /* IRQ27 */
    (uint32_t)&USART2_IRQHandler, /* IRQ28 - USART2 */
    (uint32_t)&Default_Handler,   /* IRQ29 */
    (uint32_t)&Default_Handler,   /* IRQ30 */
    (uint32_t)&Default_Handler    /* IRQ31 */
};

void Default_Handler(void) {
  while (1)
    ;
}