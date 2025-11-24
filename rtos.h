/** @file rtos.h
 * @brief Complete RTOS API - All Dissertation Features
 * M.Tech Dissertation by Josyula Sri Hari Shankar Sharma (2023HT01571)
 */

#ifndef RTOS_H
#define RTOS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Configuration */
#define MAX_TASKS           8
#define MAX_PRIORITIES      8
#define STACK_SIZE          1024
#define TICK_RATE_MS        1

#define MEM_POOL_32_SIZE    20
#define MEM_POOL_64_SIZE    15
#define MEM_POOL_128_SIZE   10
#define MEM_POOL_256_SIZE   5

#define STACK_CANARY        0xDEADBEEF

#define ENABLE_TICKLESS_IDLE    1
#define MIN_TICKLESS_TICKS      10

/* Task states */
typedef enum {
    TASK_READY = 0,
    TASK_RUNNING,
    TASK_BLOCKED,
    TASK_SUSPENDED
} task_state_t;

/* Task Control Block */
typedef struct tcb_t {
    uint32_t *stack_ptr;
    uint8_t priority;
    uint8_t base_priority;
    task_state_t state;
    struct tcb_t *next;
    uint32_t block_ticks;
    uint32_t *stack_base;
    uint32_t stack_size;
    void *blocking_object;
    uint8_t inherited_priority;
    struct tcb_t *blocked_by;
    char name[16];
} tcb_t;

/* Semaphore */
typedef struct {
    int32_t count;
    tcb_t *waiting_list;
} rtos_semaphore_t;

/* Mutex */
typedef struct {
    tcb_t *owner;
    int32_t lock_count;
    tcb_t *waiting_list;
    uint8_t original_priority;
} rtos_mutex_t;

/* Message Queue */
typedef struct {
    void *buffer;
    size_t item_size;
    size_t capacity;
    volatile size_t head;
    volatile size_t tail;
    volatile size_t count;
    rtos_semaphore_t items_available;
    rtos_semaphore_t slots_available;
    rtos_mutex_t mutex;
} rtos_queue_t;

/* Memory Pool */
typedef struct mem_block {
    struct mem_block *next;
} mem_block_t;

typedef struct {
    void *pool_start;
    size_t block_size;
    size_t num_blocks;
    mem_block_t *free_list;
    volatile uint32_t alloc_count;
} mem_pool_t;

/* Hardware Abstraction Layer */
typedef struct {
    void (*systick_init)(uint32_t ticks_per_ms);
    void (*systick_stop)(void);
    void (*systick_restart)(uint32_t ticks);
    void (*trigger_pendsv)(void);
    void (*disable_irq)(void);
    void (*enable_irq)(void);
    void (*enter_sleep)(void);
    uint32_t (*get_systick_value)(void);
} hal_interface_t;

extern hal_interface_t hal;

/* Public API */
void rtos_init(void);
bool rtos_task_create(void (*task_handler)(void), uint8_t priority, const char *name);
void rtos_start(void) __attribute__((noreturn));
void rtos_delay(uint32_t ticks);
uint32_t rtos_get_tick(void);
void rtos_task_suspend(tcb_t *task);
void rtos_task_resume(tcb_t *task);

void rtos_semaphore_init(rtos_semaphore_t *sem, int32_t initial_count);
void rtos_semaphore_wait(rtos_semaphore_t *sem);
void rtos_semaphore_signal(rtos_semaphore_t *sem);
bool rtos_semaphore_try_wait(rtos_semaphore_t *sem);

void rtos_mutex_init(rtos_mutex_t *mutex);
void rtos_mutex_lock(rtos_mutex_t *mutex);
void rtos_mutex_unlock(rtos_mutex_t *mutex);
bool rtos_mutex_try_lock(rtos_mutex_t *mutex);

bool rtos_queue_init(rtos_queue_t *queue, void *buffer, size_t item_size, size_t capacity);
bool rtos_queue_send(rtos_queue_t *queue, const void *item);
bool rtos_queue_receive(rtos_queue_t *queue, void *item);
size_t rtos_queue_count(rtos_queue_t *queue);
bool rtos_queue_is_empty(rtos_queue_t *queue);
bool rtos_queue_is_full(rtos_queue_t *queue);

void* rtos_mem_alloc(size_t size);
void rtos_mem_free(void *ptr);
void rtos_mem_stats(void);

tcb_t* rtos_check_stack_overflow(void);

void rtos_tickless_enable(bool enable);
uint32_t rtos_get_sleep_ticks(void);

void rtos_scheduler_run(void);

#endif /* RTOS_H */