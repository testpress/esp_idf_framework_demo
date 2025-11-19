#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Heap statistics structure
typedef struct {
    uint32_t total_heap;
    uint32_t free_heap;
    uint32_t min_free_heap;
    uint32_t largest_free_block;
    uint32_t internal_free_heap;
    uint32_t internal_min_free_heap;
} heap_stats_t;

// Task statistics structure
typedef struct {
    char task_name[16];
    uint32_t stack_high_water_mark;
    UBaseType_t priority;
    eTaskState state;
    uint32_t runtime_counter;
} task_stats_t;

// Function declarations
void diagnostics_init(void);
void diagnostics_print_heap_stats(void);
void diagnostics_print_task_stats(void);
void diagnostics_get_heap_stats(heap_stats_t *stats);
void diagnostics_get_task_stats(task_stats_t *stats, uint32_t max_tasks, uint32_t *num_tasks);
uint32_t diagnostics_get_heap_usage_percentage(void);

#endif // DIAGNOSTICS_H
