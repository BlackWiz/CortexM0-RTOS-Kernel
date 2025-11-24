.syntax unified
    .cpu cortex-m0plus
    .thumb

    .global PendSV_Handler
    .global current_tcb
    
    .extern rtos_scheduler_run

    .text
    .thumb_func

PendSV_Handler:
    cpsid   i
    
    ldr     r0, =current_tcb
    ldr     r1, [r0]
    
    cmp     r1, #0
    beq     PendSV_restore
    
    mrs     r0, psp
    
    subs    r0, r0, #16
    stmia   r0!, {r4-r7}
    subs    r0, r0, #16
    
    mov     r4, r8
    mov     r5, r9
    mov     r6, r10
    mov     r7, r11
    subs    r0, r0, #16
    stmia   r0!, {r4-r7}
    subs    r0, r0, #16
    
    str     r0, [r1]
    
    push    {lr}
    bl      rtos_scheduler_run
    pop     {r0}
    mov     lr, r0
    
PendSV_restore:
    ldr     r0, =current_tcb
    ldr     r1, [r0]
    
    ldr     r0, [r1]
    
    ldmia   r0!, {r4-r7}
    mov     r8, r4
    mov     r9, r5
    mov     r10, r6
    mov     r11, r7
    
    ldmia   r0!, {r4-r7}
    
    msr     psp, r0
    
    cpsie   i
    
    ldr     r0, =0xFFFFFFFD
    bx      r0

    .size   PendSV_Handler, .-PendSV_Handler
    .end