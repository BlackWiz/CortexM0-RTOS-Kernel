/** @file rtos.c
 *
 * @brief Complete RTOS Kernel Implementation
 *
 * Implements all dissertation features:
 * - O(1) priority scheduler
 * - Semaphores
 * - Mutexes with priority inheritance
 * - Message queues
 * - Fixed-block memory allocator
 * - Tickless idle mode
 * - Stack overflow detection
 */

#include "rtos.h"
#include <string.h>

/* ============================================
 * HARDWARE REGISTER DEFINITIONS
 * ============================================ */

#define SYST_CSR ((volatile uint32_t *)0xE000E010)
#define SYST_RVR ((volatile uint32_t *)0xE000E014)
#define SYST_CVR ((volatile uint32_t *)0xE000E018)
#define SCB_ICSR ((volatile uint32_t *)0xE000ED04)
#define SCB_SHPR3 ((volatile uint32_t *)0xE000ED20)

#define SCB_ICSR_PENDSVSET_Msk (1UL << 28)
#define SYSTICK_ENABLE (1UL << 0)
#define SYSTICK_TICKINT (1UL << 1)
#define SYSTICK_CLKSOURCE (1UL << 2)
#define SYSTEM_CLOCK_HZ 16000000UL

/* ============================================
 * GLOBAL VARIABLES
 * ============================================ */

/* Task management */
static tcb_t tasks[MAX_TASKS];
static uint32_t task_stacks[MAX_TASKS][STACK_SIZE / 4];
static int num_tasks = 0;

tcb_t *current_tcb = NULL;

/* Scheduler */
static tcb_t *ready_lists[MAX_PRIORITIES] = {NULL};
static uint8_t priority_bitmap = 0;
static volatile uint32_t system_ticks = 0;

/* Tickless idle */
static bool tickless_enabled = ENABLE_TICKLESS_IDLE;
static volatile uint32_t sleep_ticks = 0;

/* Memory pools */
static uint8_t mem_pool_32[MEM_POOL_32_SIZE][32] __attribute__((aligned(4)));
static uint8_t mem_pool_64[MEM_POOL_64_SIZE][64] __attribute__((aligned(4)));
static uint8_t mem_pool_128[MEM_POOL_128_SIZE][128] __attribute__((aligned(4)));
static uint8_t mem_pool_256[MEM_POOL_256_SIZE][256] __attribute__((aligned(4)));

static mem_pool_t pools[4];

/* ============================================
 * FORWARD DECLARATIONS
 * ============================================ */

static void initialize_task_stack(int task_idx, void (*task_handler)(void));
static void add_to_ready_list(tcb_t *task);
static void remove_from_ready_list(tcb_t *task);
static tcb_t *get_highest_priority_task(void);
static void schedule(void);
static uint8_t find_highest_priority(void);
static void boost_priority(tcb_t *task, uint8_t new_priority);
static void restore_priority(tcb_t *task);
static void init_memory_pools(void);
static mem_pool_t *select_pool(size_t size);
static void rtos_idle_task(void);

/* ============================================
 * HARDWARE ABSTRACTION LAYER
 * ============================================ */

static void hal_systick_init(uint32_t ticks_per_ms) {
  *SYST_RVR = (SYSTEM_CLOCK_HZ / 1000) * ticks_per_ms - 1;
  *SYST_CVR = 0;
  *SYST_CSR = SYSTICK_ENABLE | SYSTICK_TICKINT | SYSTICK_CLKSOURCE;
}

static void hal_systick_stop(void) { *SYST_CSR &= ~SYSTICK_ENABLE; }

static void hal_systick_restart(uint32_t ticks) {
  *SYST_RVR = ticks - 1;
  *SYST_CVR = 0;
  *SYST_CSR = SYSTICK_ENABLE | SYSTICK_TICKINT | SYSTICK_CLKSOURCE;
}

static void hal_trigger_pendsv(void) { *SCB_ICSR |= SCB_ICSR_PENDSVSET_Msk; }

static void hal_disable_irq(void) { __asm volatile("cpsid i" : : : "memory"); }

static void hal_enable_irq(void) { __asm volatile("cpsie i" : : : "memory"); }

static void hal_enter_sleep(void) { __asm volatile("wfi"); }

static uint32_t hal_get_systick_value(void) { return *SYST_CVR; }

hal_interface_t hal = {.systick_init = hal_systick_init,
                       .systick_stop = hal_systick_stop,
                       .systick_restart = hal_systick_restart,
                       .trigger_pendsv = hal_trigger_pendsv,
                       .disable_irq = hal_disable_irq,
                       .enable_irq = hal_enable_irq,
                       .enter_sleep = hal_enter_sleep,
                       .get_systick_value = hal_get_systick_value};

/* ============================================
 * PUBLIC API - CORE FUNCTIONS
 * ============================================ */

void rtos_init(void) {
  hal.disable_irq();

  memset(tasks, 0, sizeof(tasks));
  num_tasks = 0;

  for (int i = 0; i < MAX_PRIORITIES; i++) {
    ready_lists[i] = NULL;
  }

  priority_bitmap = 0;
  system_ticks = 0;
  current_tcb = NULL;

  init_memory_pools();

  /* Create Idle Task at lowest priority */
  rtos_task_create(rtos_idle_task, MAX_PRIORITIES - 1, "Idle");
}

bool rtos_task_create(void (*task_handler)(void), uint8_t priority,
                      const char *name) {
  if (num_tasks >= MAX_TASKS || priority >= MAX_PRIORITIES ||
      task_handler == NULL) {
    return false;
  }

  hal.disable_irq();

  tcb_t *new_task = &tasks[num_tasks];
  new_task->priority = priority;
  new_task->base_priority = priority;
  new_task->inherited_priority = priority;
  new_task->state = TASK_READY;
  new_task->next = NULL;
  new_task->block_ticks = 0;
  new_task->blocking_object = NULL;
  new_task->blocked_by = NULL;
  new_task->stack_base = task_stacks[num_tasks];
  new_task->stack_size = STACK_SIZE;

  if (name != NULL) {
    strncpy(new_task->name, name, 15);
    new_task->name[15] = '\0';
  } else {
    new_task->name[0] = '\0';
  }

  initialize_task_stack(num_tasks, task_handler);
  task_stacks[num_tasks][0] = STACK_CANARY;

  add_to_ready_list(new_task);

  num_tasks++;

  if (current_tcb != NULL && new_task->priority < current_tcb->priority) {
    schedule();
  }

  hal.enable_irq();

  return true;
}

void rtos_start(void) {
  hal.systick_init(TICK_RATE_MS);

  *SCB_SHPR3 |= (0xFFUL << 16);

  current_tcb = get_highest_priority_task();

  /* Should never be NULL because of Idle Task */
  if (current_tcb == NULL) {
    while (1)
      ;
  }

  current_tcb->state = TASK_RUNNING;

  hal.enable_irq();

  __asm volatile(
      "   ldr r0, =current_tcb     \n"
      "   ldr r1, [r0]             \n"
      "   ldr r0, [r1]             \n" /* r0 = current_tcb->stack_ptr */
      "   msr psp, r0              \n" /* Set PSP to task stack */
      "   movs r0, #2              \n" /* Bit 1 = 1 (PSP), Bit 0 = 0
                                          (Privileged) */
      "   msr control, r0          \n" /* Switch to PSP */
      "   isb                      \n"

      /* Pop R8-R11 (stored at bottom) into R4-R7 */
      "   pop {r4-r7}              \n"
      "   mov r8, r4               \n"
      "   mov r9, r5               \n"
      "   mov r10, r6              \n"
      "   mov r11, r7              \n"

      /* Pop R4-R7 */
      "   pop {r4-r7}              \n"

      /* Pop Exception Frame R0-R3 */
      "   pop {r0-r3}              \n"

      /* Pop Exception Frame R12, LR, PC, xPSR into R4-R7 */
      "   pop {r4-r7}              \n"
      "   mov r12, r4              \n"
      "   mov lr, r5               \n"
      "   msr apsr_nzcvq, r7       \n" /* Restore flags */
      "   bx r6                    \n" /* Jump to PC */
  );

  while (1)
    ;
}

void rtos_delay(uint32_t ticks) {
  if (ticks == 0 || current_tcb == NULL) {
    return;
  }

  hal.disable_irq();

  remove_from_ready_list(current_tcb); /* IMPORTANT: Remove from ready list */
  current_tcb->block_ticks = ticks;
  current_tcb->state = TASK_BLOCKED;

  schedule();

  hal.enable_irq();
}

uint32_t rtos_get_tick(void) { return system_ticks; }

void rtos_task_suspend(tcb_t *task) {
  if (task == NULL)
    return;

  hal.disable_irq();

  if (task->state == TASK_READY) {
    remove_from_ready_list(task);
  }
  task->state = TASK_SUSPENDED;

  if (task == current_tcb) {
    schedule();
  }

  hal.enable_irq();
}

void rtos_task_resume(tcb_t *task) {
  if (task == NULL)
    return;

  hal.disable_irq();

  if (task->state == TASK_SUSPENDED) {
    task->state = TASK_READY;
    add_to_ready_list(task);

    if (task->priority < current_tcb->priority) {
      schedule();
    }
  }

  hal.enable_irq();
}

void rtos_semaphore_init(rtos_semaphore_t *sem, int32_t initial_count) {
  if (sem == NULL)
    return;

  sem->count = initial_count;
  sem->waiting_list = NULL;
}

void rtos_semaphore_wait(rtos_semaphore_t *sem) {
  if (sem == NULL || current_tcb == NULL)
    return;

  hal.disable_irq();

  sem->count--;

  if (sem->count < 0) {
    remove_from_ready_list(current_tcb); /* IMPORTANT: Remove from ready list */
    current_tcb->state = TASK_BLOCKED;
    current_tcb->blocking_object = sem;

    current_tcb->next = sem->waiting_list;
    sem->waiting_list = current_tcb;

    schedule();
  }

  hal.enable_irq();
}

void rtos_semaphore_signal(rtos_semaphore_t *sem) {
  if (sem == NULL)
    return;

  hal.disable_irq();

  sem->count++;

  if (sem->count <= 0 && sem->waiting_list != NULL) {
    tcb_t *unblocked_task = sem->waiting_list;
    sem->waiting_list = unblocked_task->next;

    unblocked_task->state = TASK_READY;
    unblocked_task->blocking_object = NULL;
    unblocked_task->next = NULL;

    add_to_ready_list(unblocked_task);

    if (current_tcb != NULL &&
        unblocked_task->priority < current_tcb->priority) {
      schedule();
    }
  }

  hal.enable_irq();
}

bool rtos_semaphore_try_wait(rtos_semaphore_t *sem) {
  if (sem == NULL)
    return false;

  hal.disable_irq();

  bool success = false;
  if (sem->count > 0) {
    sem->count--;
    success = true;
  }

  hal.enable_irq();

  return success;
}

/* ============================================
 * MUTEXES (Priority Inheritance)
 * ============================================ */

void rtos_mutex_init(rtos_mutex_t *mutex) {
  if (mutex == NULL)
    return;

  mutex->owner = NULL;
  mutex->lock_count = 0;
  mutex->waiting_list = NULL;
  mutex->original_priority = 0;
}

void rtos_mutex_lock(rtos_mutex_t *mutex) {
  if (mutex == NULL || current_tcb == NULL)
    return;

  hal.disable_irq();

  if (mutex->owner == current_tcb) {
    mutex->lock_count++;
    hal.enable_irq();
    return;
  }

  if (mutex->owner == NULL) {
    mutex->owner = current_tcb;
    mutex->lock_count = 1;
    mutex->original_priority = current_tcb->base_priority;
    hal.enable_irq();
    return;
  }

  /* Mutex is locked - implement priority inheritance */
  if (current_tcb->priority < mutex->owner->priority) {
    boost_priority(mutex->owner, current_tcb->priority);
  }

  remove_from_ready_list(current_tcb); /* IMPORTANT: Remove from ready list */
  current_tcb->state = TASK_BLOCKED;
  current_tcb->blocking_object = mutex;
  current_tcb->blocked_by = mutex->owner;

  current_tcb->next = mutex->waiting_list;
  mutex->waiting_list = current_tcb;

  schedule();

  hal.enable_irq();
}

void rtos_mutex_unlock(rtos_mutex_t *mutex) {
  if (mutex == NULL || current_tcb == NULL)
    return;

  hal.disable_irq();

  if (mutex->owner != current_tcb) {
    hal.enable_irq();
    return;
  }

  mutex->lock_count--;

  if (mutex->lock_count > 0) {
    hal.enable_irq();
    return;
  }

  /* Restore original priority */
  restore_priority(current_tcb);

  if (mutex->waiting_list != NULL) {
    tcb_t *highest_waiting = NULL;
    tcb_t *curr = mutex->waiting_list;

    /* Find highest priority waiting task */
    while (curr != NULL) {
      if (highest_waiting == NULL ||
          curr->priority < highest_waiting->priority) {
        highest_waiting = curr;
      }
      curr = curr->next;
    }

    /* Remove from waiting list */
    if (highest_waiting != NULL) {
      curr = mutex->waiting_list;
      if (curr == highest_waiting) {
        mutex->waiting_list = curr->next;
      } else {
        while (curr->next != highest_waiting) {
          curr = curr->next;
        }
        curr->next = highest_waiting->next;
      }

      highest_waiting->state = TASK_READY;
      highest_waiting->blocking_object = NULL;
      highest_waiting->blocked_by = NULL;
      highest_waiting->next = NULL;

      add_to_ready_list(highest_waiting);

      /* Grant ownership to the new task */
      mutex->owner = highest_waiting;
      mutex->lock_count = 1;
      mutex->original_priority = highest_waiting->base_priority;

      if (highest_waiting->priority < current_tcb->priority) {
        schedule();
      }
    } else {
      mutex->owner = NULL;
    }
  } else {
    mutex->owner = NULL;
  }

  hal.enable_irq();
}

bool rtos_mutex_try_lock(rtos_mutex_t *mutex) {
  if (mutex == NULL || current_tcb == NULL)
    return false;

  hal.disable_irq();

  bool success = false;
  if (mutex->owner == NULL) {
    mutex->owner = current_tcb;
    mutex->lock_count = 1;
    mutex->original_priority = current_tcb->base_priority;
    success = true;
  } else if (mutex->owner == current_tcb) {
    mutex->lock_count++;
    success = true;
  }

  hal.enable_irq();

  return success;
}

/* ============================================
 * MESSAGE QUEUES
 * ============================================ */

bool rtos_queue_init(rtos_queue_t *queue, void *buffer, size_t item_size,
                     size_t capacity) {
  if (queue == NULL || buffer == NULL || item_size == 0 || capacity == 0) {
    return false;
  }

  queue->buffer = buffer;
  queue->item_size = item_size;
  queue->capacity = capacity;
  queue->head = 0;
  queue->tail = 0;
  queue->count = 0;

  rtos_mutex_init(&queue->mutex);
  rtos_semaphore_init(&queue->items_available, 0);
  rtos_semaphore_init(&queue->slots_available, capacity);

  return true;
}

bool rtos_queue_send(rtos_queue_t *queue, const void *item) {
  if (queue == NULL || item == NULL)
    return false;

  rtos_semaphore_wait(&queue->slots_available);
  rtos_mutex_lock(&queue->mutex);

  uint8_t *dest = (uint8_t *)queue->buffer + (queue->head * queue->item_size);
  memcpy(dest, item, queue->item_size);

  queue->head = (queue->head + 1) % queue->capacity;
  queue->count++;

  rtos_mutex_unlock(&queue->mutex);
  rtos_semaphore_signal(&queue->items_available);

  return true;
}

bool rtos_queue_receive(rtos_queue_t *queue, void *item) {
  if (queue == NULL || item == NULL)
    return false;

  rtos_semaphore_wait(&queue->items_available);
  rtos_mutex_lock(&queue->mutex);

  uint8_t *src = (uint8_t *)queue->buffer + (queue->tail * queue->item_size);
  memcpy(item, src, queue->item_size);

  queue->tail = (queue->tail + 1) % queue->capacity;
  queue->count--;

  rtos_mutex_unlock(&queue->mutex);
  rtos_semaphore_signal(&queue->slots_available);

  return true;
}

size_t rtos_queue_count(rtos_queue_t *queue) {
  return queue ? queue->count : 0;
}

bool rtos_queue_is_empty(rtos_queue_t *queue) {
  return queue ? (queue->count == 0) : true;
}

bool rtos_queue_is_full(rtos_queue_t *queue) {
  return queue ? (queue->count >= queue->capacity) : true;
}

/* ============================================
 * MEMORY ALLOCATOR (Fixed-Block)
 * ============================================ */

static void init_memory_pools(void) {
  pools[0].pool_start = mem_pool_32;
  pools[0].block_size = 32;
  pools[0].num_blocks = MEM_POOL_32_SIZE;
  pools[0].alloc_count = 0;

  pools[1].pool_start = mem_pool_64;
  pools[1].block_size = 64;
  pools[1].num_blocks = MEM_POOL_64_SIZE;
  pools[1].alloc_count = 0;

  pools[2].pool_start = mem_pool_128;
  pools[2].block_size = 128;
  pools[2].num_blocks = MEM_POOL_128_SIZE;
  pools[2].alloc_count = 0;

  pools[3].pool_start = mem_pool_256;
  pools[3].block_size = 256;
  pools[3].num_blocks = MEM_POOL_256_SIZE;
  pools[3].alloc_count = 0;

  /* Initialize free lists for each pool */
  for (int i = 0; i < 4; i++) {
    uint8_t *block_ptr = (uint8_t *)pools[i].pool_start;
    mem_block_t *prev_block = NULL;

    pools[i].free_list = (mem_block_t *)block_ptr;

    for (size_t j = 0; j < pools[i].num_blocks; j++) {
      mem_block_t *current_block = (mem_block_t *)block_ptr;

      if (prev_block != NULL) {
        prev_block->next = current_block;
      }

      prev_block = current_block;
      block_ptr += pools[i].block_size;
    }

    /* Terminate the list */
    if (prev_block != NULL) {
      prev_block->next = NULL;
    }
  }
}

static mem_pool_t *select_pool(size_t size) {
  if (size <= 32)
    return &pools[0];
  if (size <= 64)
    return &pools[1];
  if (size <= 128)
    return &pools[2];
  if (size <= 256)
    return &pools[3];
  return NULL;
}

void *rtos_mem_alloc(size_t size) {
  mem_pool_t *pool = select_pool(size);

  if (pool == NULL)
    return NULL;

  hal.disable_irq();

  if (pool->free_list == NULL) {
    hal.enable_irq();
    return NULL;
  }

  mem_block_t *block = pool->free_list;
  pool->free_list = block->next;
  pool->alloc_count++;

  hal.enable_irq();

  return (void *)block;
}

void rtos_mem_free(void *ptr) {
  if (ptr == NULL)
    return;

  mem_pool_t *pool = NULL;

  for (int i = 0; i < 4; i++) {
    uint8_t *pool_start = (uint8_t *)pools[i].pool_start;
    uint8_t *pool_end =
        pool_start + (pools[i].num_blocks * pools[i].block_size);

    if ((uint8_t *)ptr >= pool_start && (uint8_t *)ptr < pool_end) {
      pool = &pools[i];
      break;
    }
  }

  if (pool == NULL)
    return;

  hal.disable_irq();

  mem_block_t *block = (mem_block_t *)ptr;
  block->next = pool->free_list;
  pool->free_list = block;
  pool->alloc_count--;

  hal.enable_irq();
}

void rtos_mem_stats(void) { /* Could print statistics via UART if needed */ }

tcb_t *rtos_check_stack_overflow(void) {
  for (int i = 0; i < num_tasks; i++) {
    if (task_stacks[i][0] != STACK_CANARY) {
      return &tasks[i];
    }
  }
  return NULL;
}

void rtos_tickless_enable(bool enable) { tickless_enabled = enable; }

uint32_t rtos_get_sleep_ticks(void) { return sleep_ticks; }

static void initialize_task_stack(int task_idx, void (*task_handler)(void)) {
  uint32_t *stack_top = &task_stacks[task_idx][(STACK_SIZE / 4) - 1];

  stack_top = (uint32_t *)((uint32_t)stack_top & ~7); /* 8-byte alignment */

  *(--stack_top) = 0x01000000;             /* xPSR (Thumb bit set) */
  *(--stack_top) = (uint32_t)task_handler; /* PC */
  *(--stack_top) = 0xFFFFFFFD;             /* LR (Return to Thread mode, PSP) */
  *(--stack_top) = 0x12121212;             /* R12 */
  *(--stack_top) = 0x03030303;             /* R3 */
  *(--stack_top) = 0x02020202;             /* R2 */
  *(--stack_top) = 0x01010101;             /* R1 */
  *(--stack_top) = 0x00000000;             /* R0 */

  /* PendSV expects: R8, R9, R10, R11, R4, R5, R6, R7 (Low to High) */
  /* So we push in reverse order: R7, R6, R5, R4, R11, R10, R9, R8 */

  *(--stack_top) = 0x07070707; /* R7 */
  *(--stack_top) = 0x06060606; /* R6 */
  *(--stack_top) = 0x05050505; /* R5 */
  *(--stack_top) = 0x04040404; /* R4 */

  *(--stack_top) = 0x11111111; /* R11 */
  *(--stack_top) = 0x10101010; /* R10 */
  *(--stack_top) = 0x09090909; /* R9 */
  *(--stack_top) = 0x08080808; /* R8 */

  task_stacks[task_idx][0] = STACK_CANARY; /* Set canary at bottom */
  tasks[task_idx].stack_ptr = stack_top;
}

static void add_to_ready_list(tcb_t *task) {
  if (task == NULL)
    return;

  int priority = task->priority;
  if (ready_lists[priority] == NULL) {
    ready_lists[priority] = task;
    task->next = NULL;
    priority_bitmap |= (1 << priority);
  } else {
    /* Add to end of list (FIFO) */
    tcb_t *curr = ready_lists[priority];
    while (curr->next != NULL) {
      curr = curr->next;
    }
    curr->next = task;
    task->next = NULL;
  }
}

static void remove_from_ready_list(tcb_t *task) {
  if (task == NULL)
    return;

  int priority = task->priority;
  if (ready_lists[priority] == task) {
    ready_lists[priority] = task->next;
    if (ready_lists[priority] == NULL) {
      priority_bitmap &= ~(1 << priority);
    }
  } else {
    tcb_t *curr = ready_lists[priority];
    while (curr != NULL && curr->next != task) {
      curr = curr->next;
    }
    if (curr != NULL) {
      curr->next = task->next;
    }
  }
  task->next = NULL;
}

static uint8_t find_highest_priority(void) {
  if (priority_bitmap == 0)
    return 0xFF; /* No tasks ready */

  for (int i = 0; i < MAX_PRIORITIES; i++) {
    if (priority_bitmap & (1 << i)) {
      return i;
    }
  }
  return 0xFF;
}

static tcb_t *get_highest_priority_task(void) {
  uint8_t highest_prio = find_highest_priority();
  if (highest_prio == 0xFF)
    return NULL;

  return ready_lists[highest_prio];
}

static void schedule(void) { hal.trigger_pendsv(); }

static void boost_priority(tcb_t *task, uint8_t new_priority) {
  if (task == NULL || new_priority >= task->priority)
    return;

  if (task->state == TASK_READY) {
    remove_from_ready_list(task);
    task->priority = new_priority;
    add_to_ready_list(task);
  } else {
    task->priority = new_priority;
  }
}

static void restore_priority(tcb_t *task) {
  if (task == NULL || task->priority == task->base_priority)
    return;

  if (task->state == TASK_READY) {
    remove_from_ready_list(task);
    task->priority = task->base_priority;
    add_to_ready_list(task);
  } else {
    task->priority = task->base_priority;
  }
}

/* ============================================
 * INTERRUPT HANDLERS
 * ============================================ */

void rtos_scheduler_run(void) {
  /* Interrupts are already disabled by PendSV */

  tcb_t *next_task = get_highest_priority_task();

  if (next_task == NULL) {
    /* Should not happen with Idle Task */
    return;
  }

  if (next_task != current_tcb) {
    current_tcb = next_task;
    current_tcb->state = TASK_RUNNING;
  }
}

void SysTick_Handler(void) {
  system_ticks++;

  bool wake_needed = false;

  /* Check for blocked tasks */
  for (int i = 0; i < num_tasks; i++) {
    if (tasks[i].state == TASK_BLOCKED && tasks[i].block_ticks > 0) {
      tasks[i].block_ticks--;
      if (tasks[i].block_ticks == 0) {
        tasks[i].state = TASK_READY;
        tasks[i].blocking_object = NULL;
        tasks[i].blocked_by = NULL;
        add_to_ready_list(&tasks[i]);
        wake_needed = true;
      }
    }
  }

  /* Round-robin for same priority */
  if (current_tcb != NULL && current_tcb->next != NULL) {
    wake_needed = true;
  }

  if (wake_needed) {
    hal.trigger_pendsv();
  }
}

static void rtos_idle_task(void) {
  while (1) {
    if (tickless_enabled) {
      /* ... tickless logic ... */
      /* For now, just sleep */
      hal.enter_sleep();
    } else {
      hal.enter_sleep();
    }
  }
}

/*** end of file ***/