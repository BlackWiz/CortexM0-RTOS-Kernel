/** @file main.c
 *
 * @brief Complete RTOS Demonstration - UART Version
 *
 * Demonstrates ALL dissertation features via UART output:
 * 1. Priority-based preemptive scheduling
 * 2. Semaphores for synchronization
 * 3. Mutexes with priority inheritance
 * 4. Message queues for inter-task communication
 * 5. Fixed-block memory allocator
 * 6. Stack overflow detection
 * 7. Tickless idle mode
 *
 * @par Dissertation Project
 * M.Tech - Building a Real-Time Operating System
 * Student: Josyula Sri Hari Shankar Sharma (2023HT01571)
 */

#include "rtos.h"
#include "uart.h"
#include <stdint.h>
#include <string.h>

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
rtos_semaphore_t print_semaphore;   /* Protects UART access */
rtos_mutex_t shared_resource_mutex; /* For priority inheritance demo */
rtos_queue_t message_queue;         /* Producer-consumer queue */

/* Queue buffer */
#define QUEUE_SIZE 5
static message_t queue_buffer[QUEUE_SIZE];

/* Shared resource for priority inversion demo */
static volatile uint32_t shared_counter = 0;

/* Helper to print integers */
void uart_print_int(int val) {
  char buffer[12];
  int i = 0;
  if (val == 0) {
    uart_print("0");
    return;
  }
  if (val < 0) {
    uart_print("-");
    val = -val;
  }
  while (val > 0) {
    buffer[i++] = (val % 10) + '0';
    val /= 10;
  }
  for (int j = i - 1; j >= 0; j--) {
    char c[2] = {buffer[j], '\0'};
    uart_print(c);
  }
}

/* ============================================
 * TASK 1: PRODUCER (Priority 1 - High)
 * Demonstrates: Message Queue Send, Memory Allocation
 * ============================================ */

void task_producer(void) {
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

      /* Print status */
      rtos_semaphore_wait(&print_semaphore);
      uart_print("[PRODUCER] Sent Msg #");
      uart_print_int(msg->sequence_number);
      uart_print("\r\n");
      rtos_semaphore_signal(&print_semaphore);
    } else {
      rtos_semaphore_wait(&print_semaphore);
      uart_print("[PRODUCER] Alloc Failed!\r\n");
      rtos_semaphore_signal(&print_semaphore);
    }

    rtos_delay(1000); /* Produce every 1s */
  }
}

/* ============================================
 * TASK 2: CONSUMER (Priority 2 - Medium)
 * Demonstrates: Message Queue Receive, Semaphores
 * ============================================ */

void task_consumer(void) {
  message_t received_msg;

  while (1) {
    /* Receive message from queue (blocks if empty) */
    if (rtos_queue_receive(&message_queue, &received_msg)) {
      /* Process message (simulate work) */
      for (volatile int i = 0; i < 10000; i++)
        ;

      /* Print status */
      rtos_semaphore_wait(&print_semaphore);
      uart_print("  [CONSUMER] Recv Msg #");
      uart_print_int(received_msg.sequence_number);
      uart_print(" (Tick: ");
      uart_print_int(received_msg.timestamp);
      uart_print(")\r\n");
      rtos_semaphore_signal(&print_semaphore);
    }
  }
}

/* ============================================
 * TASK 3: PRIORITY INHERITANCE DEMO (Priority 3 - Low)
 * Demonstrates: Mutex with Priority Inheritance
 * ============================================ */

void task_low_priority(void) {
  while (1) {
    /* Acquire mutex (holds it for a "long" time) */
    rtos_mutex_lock(&shared_resource_mutex);

    /* Simulate critical section work */
    shared_counter++;

    rtos_semaphore_wait(&print_semaphore);
    uart_print("    [LOW PRIO] Holding Mutex...\r\n");
    rtos_semaphore_signal(&print_semaphore);

    /* Hold mutex for extended period */
    for (volatile uint32_t i = 0; i < 500000; i++)
      ;

    rtos_mutex_unlock(&shared_resource_mutex);

    rtos_semaphore_wait(&print_semaphore);
    uart_print("    [LOW PRIO] Released Mutex\r\n");
    rtos_semaphore_signal(&print_semaphore);

    rtos_delay(5000);
  }
}

/* ============================================
 * TASK 4: MEMORY ALLOCATOR DEMO (Priority 4 - Medium-Low)
 * Demonstrates: Fixed-Block Memory Allocator
 * ============================================ */

void task_memory_demo(void) {
  void *ptrs[4];

  while (1) {
    /* Allocate various sizes to demonstrate pool selection */
    ptrs[0] = rtos_mem_alloc(20);  /* Uses 32-byte pool */
    ptrs[1] = rtos_mem_alloc(50);  /* Uses 64-byte pool */
    ptrs[2] = rtos_mem_alloc(100); /* Uses 128-byte pool */
    ptrs[3] = rtos_mem_alloc(200); /* Uses 256-byte pool */

    rtos_semaphore_wait(&print_semaphore);
    if (ptrs[0] && ptrs[1] && ptrs[2] && ptrs[3]) {
      uart_print("      [MEMORY] Alloc OK (4 blocks)\r\n");
    } else {
      uart_print("      [MEMORY] Alloc FAILED\r\n");
    }
    rtos_semaphore_signal(&print_semaphore);

    rtos_delay(100);

    /* Free all memory (O(1) deallocation) */
    for (int i = 0; i < 4; i++) {
      if (ptrs[i]) {
        rtos_mem_free(ptrs[i]);
      }
    }

    rtos_delay(2900);
  }
}

/* ============================================
 * TASK 5: MONITOR (Priority 5 - Low)
 * Demonstrates: Stack Overflow Detection
 * ============================================ */

void task_monitor(void) {
  tcb_t *faulty_task;

  while (1) {
    /* Check for stack overflow every 5 seconds */
    rtos_delay(5000);

    faulty_task = rtos_check_stack_overflow();

    if (faulty_task != NULL) {
      rtos_semaphore_wait(&print_semaphore);
      uart_print("!!! STACK OVERFLOW DETECTED !!!\r\n");
      rtos_semaphore_signal(&print_semaphore);
      while (1)
        ;
    } else {
      /* Optional: heartbeat to show monitor is alive */
      /*
      rtos_semaphore_wait(&print_semaphore);
      uart_print("[MONITOR] System Healthy\r\n");
      rtos_semaphore_signal(&print_semaphore);
      */
    }
  }
}

/* ============================================
 * TASK 6: IDLE (Priority 7 - Lowest)
 * Demonstrates: Tickless Idle Mode
 * ============================================ */

void task_idle(void) {
  while (1) {
    /* Enter low-power sleep mode */
    __asm volatile("wfi");
  }
}

/* ============================================
 * MAIN FUNCTION
 * ============================================ */

int main(void) {
  /* Initialize UART first */
  uart_init();

  /* Small delay for UART to stabilize */
  for (volatile int i = 0; i < 100000; i++)
    ;

  uart_print("\r\n\r\n*** RTOS DEMONSTRATION STARTING ***\r\n");
  uart_print("Mode: UART Output (No LEDs)\r\n");
  uart_print("Initializing Kernel...\r\n");

  /* Initialize RTOS kernel */
  rtos_init();

  /* Initialize synchronization primitives */
  rtos_semaphore_init(&print_semaphore, 1); /* Binary semaphore for UART */
  rtos_mutex_init(&shared_resource_mutex);  /* Mutex for PI demo */

  /* Initialize message queue */
  rtos_queue_init(&message_queue, queue_buffer, sizeof(message_t), QUEUE_SIZE);

  /* Enable tickless idle mode */
  rtos_tickless_enable(true);

  /* Create demonstration tasks */
  rtos_task_create(task_producer, 1, "Producer");
  rtos_task_create(task_consumer, 2, "Consumer");
  rtos_task_create(task_low_priority, 3, "Low Priority");
  rtos_task_create(task_memory_demo, 4, "Memory Demo");
  rtos_task_create(task_monitor, 5, "Monitor");
  rtos_task_create(task_idle, 7, "Idle");

  uart_print("Starting Scheduler...\r\n\r\n");

  /* Start RTOS scheduler - never returns */
  rtos_start();

  /* Should never reach here */
  while (1)
    ;

  return 0;
}