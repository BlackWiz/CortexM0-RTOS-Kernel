/** @file main.c
 *
 * @brief Complete RTOS Demonstration - All Features
 *
 * Demonstrates ALL dissertation features:
 * 1. Priority-based preemptive scheduling
 * 2. Semaphores for synchronization
 * 3. Mutexes with priority inheritance
 * 4. Message queues for inter-task communication
 * 5. Fixed-block memory allocator
 * 6. Stack overflow detection
 * 7. Tickless idle mode
 *
 * @par Hardware Setup:
 * - LED1 (Green) on PB7 - Producer task (Priority 1)
 * - LED2 (Blue) on PB6 - Consumer task (Priority 2)
 * - LED3 (Red) on PA5 - Error indicator (Stack overflow)
 * - LED4 (Yellow) on PB8 - Memory allocator demo (Priority 3)
 *
 * @par Dissertation Project
 * M.Tech - Building a Real-Time Operating System
 * Student: Josyula Sri Hari Shankar Sharma (2023HT01571)
 */

#include "rtos.h"
#include <stdint.h>
#include <string.h>

/* ============================================
 * GPIO REGISTER DEFINITIONS
 * ============================================ */

#define RCC_IOPENR      ((volatile uint32_t *)0x40021034)
#define GPIOA_MODER     ((volatile uint32_t *)0x50000000)
#define GPIOA_ODR       ((volatile uint32_t *)0x50000014)
#define GPIOB_MODER     ((volatile uint32_t *)0x50000400)
#define GPIOB_ODR       ((volatile uint32_t *)0x50000414)

/* Pin definitions */
#define LED_RED_PIN     5   /* PA5 - Error LED */
#define LED_GREEN_PIN   7   /* PB7 - Producer */
#define LED_BLUE_PIN    6   /* PB6 - Consumer */
#define LED_YELLOW_PIN  8   /* PB8 - Memory demo */

/* ============================================
 * DEMONSTRATION DATA STRUCTURES
 * ============================================ */

/* Message structure for queue demo */
typedef struct {
    uint32_t sequence_number;
    uint32_t timestamp;
    uint8_t data[16];
} message_t;

/* Global synchronization objects */
rtos_semaphore_t led_semaphore;        /* Protects LED access */
rtos_mutex_t shared_resource_mutex;    /* For priority inheritance demo */
rtos_queue_t message_queue;            /* Producer-consumer queue */

/* Queue buffer */
#define QUEUE_SIZE 5
static message_t queue_buffer[QUEUE_SIZE];

/* Shared resource for priority inversion demo */
static volatile uint32_t shared_counter = 0;

/* ============================================
 * HARDWARE INITIALIZATION
 * ============================================ */

void gpio_init(void)
{
    /* Enable GPIO clocks for Port A and Port B */
    *RCC_IOPENR |= (1 << 0) | (1 << 1);  /* GPIOAEN | GPIOBEN */
    
    /* Configure PA5 (Red LED) as output */
    *GPIOA_MODER &= ~(3UL << (LED_RED_PIN * 2));
    *GPIOA_MODER |= (1UL << (LED_RED_PIN * 2));
    
    /* Configure PB6 (Blue LED) as output */
    *GPIOB_MODER &= ~(3UL << (LED_BLUE_PIN * 2));
    *GPIOB_MODER |= (1UL << (LED_BLUE_PIN * 2));
    
    /* Configure PB7 (Green LED) as output */
    *GPIOB_MODER &= ~(3UL << (LED_GREEN_PIN * 2));
    *GPIOB_MODER |= (1UL << (LED_GREEN_PIN * 2));
    
    /* Configure PB8 (Yellow LED) as output */
    *GPIOB_MODER &= ~(3UL << (LED_YELLOW_PIN * 2));
    *GPIOB_MODER |= (1UL << (LED_YELLOW_PIN * 2));
    
    /* Turn off all LEDs initially */
    *GPIOA_ODR &= ~(1 << LED_RED_PIN);
    *GPIOB_ODR &= ~(1 << LED_GREEN_PIN);
    *GPIOB_ODR &= ~(1 << LED_BLUE_PIN);
    *GPIOB_ODR &= ~(1 << LED_YELLOW_PIN);
}

/* ============================================
 * TASK 1: PRODUCER (Priority 1 - High)
 * Demonstrates: Message Queue Send, Memory Allocation
 * ============================================ */

void task_producer(void)
{
    uint32_t seq = 0;
    
    while (1) {
        /* Create message using dynamic memory allocation */
        message_t *msg = (message_t *)rtos_mem_alloc(sizeof(message_t));
        
        if (msg != NULL) {
            msg->sequence_number = seq++;
            msg->timestamp = rtos_get_tick();
            memset(msg->data, 0xAA, sizeof(msg->data));
            
            /* Send message to queue (blocks if full) */
            rtos_queue_send(&message_queue, msg);
            
            /* Free memory after sending */
            rtos_mem_free(msg);
            
            /* Blink Green LED to indicate production */
            rtos_semaphore_wait(&led_semaphore);
            *GPIOB_ODR ^= (1 << LED_GREEN_PIN);
            rtos_semaphore_signal(&led_semaphore);
        }
        
        rtos_delay(500);  /* Produce every 500ms */
    }
}

/* ============================================
 * TASK 2: CONSUMER (Priority 2 - Medium)
 * Demonstrates: Message Queue Receive, Semaphores
 * ============================================ */

void task_consumer(void)
{
    message_t received_msg;
    
    while (1) {
        /* Receive message from queue (blocks if empty) */
        if (rtos_queue_receive(&message_queue, &received_msg)) {
            /* Process message (simulate work) */
            for (volatile int i = 0; i < 10000; i++);
            
            /* Blink Blue LED to indicate consumption */
            rtos_semaphore_wait(&led_semaphore);
            *GPIOB_ODR ^= (1 << LED_BLUE_PIN);
            rtos_semaphore_signal(&led_semaphore);
        }
    }
}

/* ============================================
 * TASK 3: PRIORITY INHERITANCE DEMO (Priority 3 - Low)
 * Demonstrates: Mutex with Priority Inheritance
 * ============================================ */

void task_low_priority(void)
{
    while (1) {
        /* Acquire mutex (holds it for a "long" time) */
        rtos_mutex_lock(&shared_resource_mutex);
        
        /* Simulate critical section work */
        shared_counter++;
        
        /* Hold mutex for extended period */
        for (volatile uint32_t i = 0; i < 100000; i++) {
            /* If high-priority task tries to acquire,
             * this task's priority will be boosted */
        }
        
        rtos_mutex_unlock(&shared_resource_mutex);
        
        rtos_delay(2000);
    }
}

/* ============================================
 * TASK 4: MEMORY ALLOCATOR DEMO (Priority 4 - Medium-Low)
 * Demonstrates: Fixed-Block Memory Allocator
 * ============================================ */

void task_memory_demo(void)
{
    void *ptrs[5];
    
    while (1) {
        /* Allocate various sizes to demonstrate pool selection */
        ptrs[0] = rtos_mem_alloc(20);   /* Uses 32-byte pool */
        ptrs[1] = rtos_mem_alloc(50);   /* Uses 64-byte pool */
        ptrs[2] = rtos_mem_alloc(100);  /* Uses 128-byte pool */
        ptrs[3] = rtos_mem_alloc(200);  /* Uses 256-byte pool */
        
        /* Blink Yellow LED to show allocator is working */
        if (ptrs[0] && ptrs[1] && ptrs[2] && ptrs[3]) {
            *GPIOB_ODR ^= (1 << LED_YELLOW_PIN);
        }
        
        rtos_delay(100);
        
        /* Free all memory (O(1) deallocation) */
        for (int i = 0; i < 4; i++) {
            if (ptrs[i]) {
                rtos_mem_free(ptrs[i]);
            }
        }
        
        rtos_delay(1900);
    }
}

/* ============================================
 * TASK 5: MONITOR (Priority 5 - Low)
 * Demonstrates: Stack Overflow Detection
 * ============================================ */

void task_monitor(void)
{
    tcb_t *faulty_task;
    
    while (1) {
        /* Check for stack overflow every 5 seconds */
        rtos_delay(5000);
        
        faulty_task = rtos_check_stack_overflow();
        
        if (faulty_task != NULL) {
            /* Stack overflow detected - critical fault */
            
            /* Turn on Red LED and blink rapidly */
            while (1) {
                *GPIOA_ODR ^= (1 << LED_RED_PIN);
                rtos_delay(100);
                
                /* System is in fault state - should reset */
            }
        }
    }
}

/* ============================================
 * TASK 6: IDLE (Priority 7 - Lowest)
 * Demonstrates: Tickless Idle Mode
 * ============================================ */

void task_idle(void)
{
    while (1) {
        /* Enter low-power sleep mode */
        /* If tickless idle enabled, SysTick stops and CPU sleeps longer */
        __asm volatile ("wfi");
    }
}

/* ============================================
 * OPTIONAL: HIGH PRIORITY TASK
 * Uncomment to demonstrate priority inheritance
 * ============================================ */

/*
void task_high_priority(void)
{
    while (1) {
        rtos_delay(1000);
        
        // Try to acquire mutex held by low-priority task
        // This will boost low-priority task's priority
        rtos_mutex_lock(&shared_resource_mutex);
        
        shared_counter += 100;
        
        rtos_mutex_unlock(&shared_resource_mutex);
    }
}
*/

/* ============================================
 * MAIN FUNCTION
 * ============================================ */

int main(void)
{
    /* Initialize hardware */
    gpio_init();
    
    /* Initialize RTOS kernel (includes memory allocator) */
    rtos_init();
    
    /* Initialize synchronization primitives */
    rtos_semaphore_init(&led_semaphore, 1);          /* Binary semaphore */
    rtos_mutex_init(&shared_resource_mutex);         /* Mutex for PI demo */
    
    /* Initialize message queue */
    rtos_queue_init(&message_queue, queue_buffer, sizeof(message_t), QUEUE_SIZE);
    
    /* Enable tickless idle mode for power saving */
    rtos_tickless_enable(true);
    
    /* Create demonstration tasks */
    rtos_task_create(task_producer, 1, "Producer");
    rtos_task_create(task_consumer, 2, "Consumer");
    rtos_task_create(task_low_priority, 3, "Low Priority");
    rtos_task_create(task_memory_demo, 4, "Memory Demo");
    rtos_task_create(task_monitor, 5, "Monitor");
    rtos_task_create(task_idle, 7, "Idle");
    
    /* Uncomment to enable priority inheritance demo */
    // rtos_task_create(task_high_priority, 0, "High Priority");
    
    /* Start RTOS scheduler - never returns */
    rtos_start();
    
    /* Should never reach here */
    while (1);
    
    return 0;
}

/* ============================================
 * EXPECTED BEHAVIOR
 * ============================================ */

/*
 * LED Patterns (during normal operation):
 * 
 * 1. GREEN LED (PB7):
 *    - Blinks every 500ms
 *    - Indicates Producer task sending messages
 * 
 * 2. BLUE LED (PB6):
 *    - Blinks when messages consumed
 *    - Slightly irregular due to queue blocking
 * 
 * 3. YELLOW LED (PB8):
 *    - Brief blink every 2 seconds
 *    - Shows memory allocator working
 * 
 * 4. RED LED (PA5):
 *    - OFF during normal operation
 *    - Rapid blink if stack overflow detected
 * 
 * Queue Behavior:
 * - Producer fills queue (max 5 messages)
 * - Consumer drains queue
 * - Demonstrates blocking on full/empty conditions
 * 
 * Memory Allocator:
 * - Allocates from 4 different pool sizes
 * - O(1) constant time allocation/deallocation
 * - No fragmentation possible
 * 
 * Priority Inheritance:
 * - Uncomment task_high_priority to see it work
 * - Low-priority task gets boosted when high-priority blocks
 * - Prevents priority inversion (Mars Pathfinder bug)
 * 
 * Tickless Idle:
 * - When all tasks sleeping, CPU enters deep sleep
 * - SysTick stops, power consumption drops
 * - Automatically wakes when task ready
 */

/*** end of file ***/