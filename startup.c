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

void Reset_Handler(void)
{
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

    while (1);
}

__attribute__((section(".isr_vector")))
const uint32_t g_pfnVectors[] = {
    (uint32_t)&_top_of_stack,
    (uint32_t)&Reset_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    0, 0, 0, 0, 0, 0, 0,
    (uint32_t)&Default_Handler,
    0, 0,
    (uint32_t)&PendSV_Handler,
    (uint32_t)&SysTick_Handler,
    
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    0,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler,
    (uint32_t)&Default_Handler
};

void Default_Handler(void)
{
    while (1);
}