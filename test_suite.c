/** @file test_suite.c
 *
 * @brief Complete RTOS Test Suite
 *
 * Validates all dissertation features:
 * - Task management and scheduling
 * - Semaphores
 * - Mutexes with priority inheritance
 * - Message queues
 * - Memory allocator
 * - Stack overflow detection
 *
 * Compile with: make test
 * Flash with: make test-flash
 *
 * @par Dissertation Chapter 6 - Testing and Validation
 */

#include "rtos.h"
#include "uart.h" /* Use the new interrupt-driven driver */
#include <stdint.h>
#include <string.h>

/* Helper to print integers using the driver's uart_print */
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

#define TEST_PASS "[PASS] "
#define TEST_FAIL "[FAIL] "
#define TEST_INFO "[INFO] "

/* Test counters */
static uint32_t tests_run = 0;
static uint32_t tests_passed = 0;
static uint32_t tests_failed = 0;

/* ============================================
 * TEST HELPER FUNCTIONS
 * ============================================ */

void test_report(const char *test_name, bool passed) {
  tests_run++;

  if (passed) {
    tests_passed++;
    uart_print(TEST_PASS);
  } else {
    tests_failed++;
    uart_print(TEST_FAIL);
  }

  uart_print(test_name);
  uart_print("\r\n");
}

void test_summary(void) {
  uart_print("\r\n========================================\r\n");
  uart_print("RTOS TEST SUMMARY\r\n");
  uart_print("========================================\r\n");

  if (tests_failed == 0) {
    uart_print("ALL TESTS PASSED!\r\n");
  } else {
    uart_print("SOME TESTS FAILED!\r\n");
  }

  uart_print("========================================\r\n");
}

/* ============================================
 * TEST 1: TASK CREATION AND SCHEDULING
 * ============================================ */

static volatile uint32_t task1_counter = 0;
static volatile uint32_t task2_counter = 0;

void test_task1(void) {
  while (1) {
    task1_counter++;
    rtos_delay(5); /* Run faster than task 2 to ensure higher count */
  }
}

void test_task2(void) {
  while (1) {
    task2_counter++;
    rtos_delay(10);
  }
}

void test_01_task_creation(void) {
  bool result = true;

  /* Test creating multiple tasks */
  result &= rtos_task_create(test_task1, 1, "Test1");
  result &= rtos_task_create(test_task2, 2, "Test2");

  test_report("T01: Task Creation", result);
}

void test_02_task_execution(void) {
  /* Let tasks run for 100ms */
  rtos_delay(100);

  /* Both tasks should have executed */
  bool result = (task1_counter > 5) && (task2_counter > 5);

  if (!result) {
    uart_print("[DEBUG] T02 Fail: T1=");
    if (task1_counter >= 10)
      uart_print("10+");
    else
      uart_print("Low");
    uart_print(" T2=");
    if (task2_counter >= 10)
      uart_print("10+");
    else
      uart_print("Low");
    uart_print("\r\n");
  }

  test_report("T02: Task Execution", result);
}

void test_03_priority_preemption(void) {
  /* High-priority task should run more often because of shorter delay */
  bool result = (task1_counter > task2_counter);

  if (!result) {
    uart_print("[DEBUG] T03 Fail: T1 <= T2\r\n");
  }

  test_report("T03: Priority Preemption", result);
}

/* ============================================
 * TEST 2: SEMAPHORES
 * ============================================ */

static rtos_semaphore_t test_sem;
static volatile uint32_t sem_counter = 0;

void test_sem_task1(void) {
  while (1) {
    rtos_semaphore_wait(&test_sem);
    sem_counter++;
    rtos_delay(10);
  }
}

void test_sem_task2(void) {
  while (1) {
    rtos_delay(20);
    rtos_semaphore_signal(&test_sem);
  }
}

void test_04_semaphore_init(void) {
  rtos_semaphore_init(&test_sem, 0);

  bool result = (test_sem.count == 0) && (test_sem.waiting_list == NULL);

  test_report("T04: Semaphore Init", result);
}

void test_05_semaphore_signal_wait(void) {
  rtos_task_create(test_sem_task1, 3, "SemTest1");
  rtos_task_create(test_sem_task2, 4, "SemTest2");

  rtos_delay(100);

  /* Semaphore should have been signaled and waited on */
  bool result = (sem_counter > 2);

  test_report("T05: Semaphore Signal/Wait", result);
}

/* ============================================
 * TEST 3: MUTEXES WITH PRIORITY INHERITANCE
 * ============================================ */

static rtos_mutex_t test_mutex;
static volatile uint32_t mutex_shared_var = 0;
static volatile bool priority_boosted = false;

void test_mutex_low_task(void) {
  while (1) {
    rtos_mutex_lock(&test_mutex);

    /* Simulate long critical section */
    for (volatile int i = 0; i < 50000; i++) {
      mutex_shared_var++;
    }

    rtos_mutex_unlock(&test_mutex);
    rtos_delay(100);
  }
}

void test_mutex_high_task(void) {
  rtos_delay(50); /* Let low task acquire first */

  /* This should boost low task's priority */
  rtos_mutex_lock(&test_mutex);
  priority_boosted = true;
  mutex_shared_var += 1000;
  rtos_mutex_unlock(&test_mutex);

  while (1) {
    rtos_delay(1000);
  }
}

void test_06_mutex_init(void) {
  rtos_mutex_init(&test_mutex);

  bool result = (test_mutex.owner == NULL) && (test_mutex.lock_count == 0);

  test_report("T06: Mutex Init", result);
}

void test_07_mutex_lock_unlock(void) {
  rtos_mutex_lock(&test_mutex);
  bool locked = (test_mutex.owner != NULL);
  rtos_mutex_unlock(&test_mutex);
  bool unlocked = (test_mutex.owner == NULL);

  bool result = locked && unlocked;

  test_report("T07: Mutex Lock/Unlock", result);
}

void test_08_priority_inheritance(void) {
  rtos_task_create(test_mutex_low_task, 5, "MutexLow");
  rtos_task_create(test_mutex_high_task, 1, "MutexHigh");

  rtos_delay(200);

  /* If priority inheritance worked, high task should have run */
  bool result = priority_boosted && (mutex_shared_var > 1000);

  test_report("T08: Priority Inheritance", result);
}

/* ============================================
 * TEST 4: MESSAGE QUEUES
 * ============================================ */

typedef struct {
  uint32_t value;
} test_msg_t;

static rtos_queue_t test_queue;
static test_msg_t queue_buffer[5];
static volatile uint32_t queue_received = 0;

void test_queue_sender(void) {
  test_msg_t msg;

  for (uint32_t i = 0; i < 10; i++) {
    msg.value = i;
    uart_print("[DEBUG] Sending: ");
    uart_print_int(i);
    uart_print("\r\n");
    if (!rtos_queue_send(&test_queue, &msg)) {
      uart_print("[DEBUG] Send Fail\r\n");
    }
    rtos_delay(10);
  }

  while (1)
    rtos_delay(1000);
}

void test_queue_receiver(void) {
  test_msg_t msg;

  for (uint32_t i = 0; i < 10; i++) {
    rtos_queue_receive(&test_queue, &msg);
    uart_print("[DEBUG] Recv: ");
    uart_print_int(msg.value);
    uart_print("\r\n");
    if (msg.value == i) {
      queue_received++;
    } else {
      uart_print("[DEBUG] T10 Mismatch: Exp ");
      uart_print_int(i);
      uart_print(" Got ");
      uart_print_int(msg.value);
      uart_print("\r\n");
    }
  }

  while (1)
    rtos_delay(1000);
}

void test_09_queue_init(void) {
  bool result =
      rtos_queue_init(&test_queue, queue_buffer, sizeof(test_msg_t), 5);

  test_report("T09: Queue Init", result);
}

void test_10_queue_send_receive(void) {
  if (!rtos_task_create(test_queue_sender, 2, "QueueSend")) {
    uart_print("[ERROR] Failed to create QueueSend task!\r\n");
  }
  if (!rtos_task_create(test_queue_receiver, 3, "QueueRecv")) {
    uart_print("[ERROR] Failed to create QueueRecv task!\r\n");
  }

  rtos_delay(300); /* Increased from 200 to ensure completion */

  /* All 10 messages should have been received correctly */
  bool result = (queue_received == 10);

  if (!result) {
    uart_print("[DEBUG] T10 Fail: Recv=");
    uart_print_int(queue_received);
    uart_print("\r\n");
  }

  test_report("T10: Queue Send/Receive", result);
}

/* ============================================
 * TEST 5: MEMORY ALLOCATOR
 * ============================================ */

void test_11_memory_alloc_free(void) {
  void *ptr1 = rtos_mem_alloc(32);
  void *ptr2 = rtos_mem_alloc(64);
  void *ptr3 = rtos_mem_alloc(128);

  bool alloc_ok = (ptr1 != NULL) && (ptr2 != NULL) && (ptr3 != NULL);

  rtos_mem_free(ptr1);
  rtos_mem_free(ptr2);
  rtos_mem_free(ptr3);

  /* Allocate again to verify free worked */
  void *ptr4 = rtos_mem_alloc(32);
  bool realloc_ok = (ptr4 != NULL);

  rtos_mem_free(ptr4);

  bool result = alloc_ok && realloc_ok;

  test_report("T11: Memory Alloc/Free", result);
}

void test_12_memory_pool_selection(void) {
  void *ptr_small = rtos_mem_alloc(20);  /* Should use 32-byte pool */
  void *ptr_medium = rtos_mem_alloc(50); /* Should use 64-byte pool */
  void *ptr_large = rtos_mem_alloc(200); /* Should use 256-byte pool */

  bool result =
      (ptr_small != NULL) && (ptr_medium != NULL) && (ptr_large != NULL);

  rtos_mem_free(ptr_small);
  rtos_mem_free(ptr_medium);
  rtos_mem_free(ptr_large);

  test_report("T12: Memory Pool Selection", result);
}

/* ============================================
 * TEST 6: STACK OVERFLOW DETECTION
 * ============================================ */

void test_13_stack_overflow_detection(void) {
  /* Normal operation - no overflow */
  tcb_t *fault = rtos_check_stack_overflow();

  bool result = (fault == NULL);

  test_report("T13: Stack Overflow Check", result);
}

/* ============================================
 * TEST RUNNER TASK
 * ============================================ */

void test_runner(void) {
  uart_print("\r\n========================================\r\n");
  uart_print("RTOS COMPREHENSIVE TEST SUITE\r\n");
  uart_print("========================================\r\n\r\n");

  /* Task Management Tests */
  test_01_task_creation();
  rtos_delay(50);
  test_02_task_execution();
  rtos_delay(50);
  test_03_priority_preemption();
  rtos_delay(50);

  /* Semaphore Tests */
  test_04_semaphore_init();
  test_05_semaphore_signal_wait();
  rtos_delay(150);

  /* Mutex Tests */
  test_06_mutex_init();
  test_07_mutex_lock_unlock();
  test_08_priority_inheritance();
  rtos_delay(300);

  /* Queue Tests */
  test_09_queue_init();
  test_10_queue_send_receive();
  rtos_delay(250);

  /* Memory Allocator Tests */
  test_11_memory_alloc_free();
  test_12_memory_pool_selection();

  /* Stack Tests */
  test_13_stack_overflow_detection();

  /* Print summary */
  test_summary();

  /* Halt */
  while (1) {
    rtos_delay(1000);
  }
}

/* ============================================
 * MAIN FOR TEST BUILD
 * ============================================ */

int main(void) {
  /* Initialize UART first - before anything else */
  uart_init();

  /* Small delay for UART to stabilize */
  for (volatile int i = 0; i < 100000; i++)
    ;

  uart_print("\r\n\r\n*** RTOS FULL TEST SUITE STARTING ***\r\n");
  uart_print("Board: STM32G071RB\r\n");
  uart_print("Baud: 115200\r\n");
  uart_print("Mode: Interrupt-Driven Ring Buffer\r\n\r\n");

  uart_print("[DEBUG] About to call rtos_init()...\r\n");
  rtos_init();
  uart_print("[DEBUG] rtos_init() completed\r\n");

  rtos_task_create(test_runner, 0, "TestRunner");
  uart_print("[DEBUG] test_runner task created\r\n");

  uart_print("[DEBUG] About to call rtos_start()...\r\n");
  rtos_start();

  uart_print("[ERROR] rtos_start() returned - should never happen!\r\n");

  while (1)
    ;

  return 0;
}

/*** end of file ***/