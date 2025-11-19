#include "diagnostics.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "diagnostics";

void diagnostics_init(void)
{
    ESP_LOGI(TAG, "Diagnostics module initialized");
}

void diagnostics_print_heap_stats(void)
{
    heap_stats_t stats;
    diagnostics_get_heap_stats(&stats);

    ESP_LOGI(TAG, "=== HEAP STATISTICS ===");
    ESP_LOGI(TAG, "Total heap: %u bytes (%.2f KB)",
             stats.total_heap, stats.total_heap / 1024.0f);
    ESP_LOGI(TAG, "Free heap: %u bytes (%.2f KB)",
             stats.free_heap, stats.free_heap / 1024.0f);
    ESP_LOGI(TAG, "Min free heap: %u bytes (%.2f KB)",
             stats.min_free_heap, stats.min_free_heap / 1024.0f);
    ESP_LOGI(TAG, "Largest free block: %u bytes (%.2f KB)",
             stats.largest_free_block, stats.largest_free_block / 1024.0f);
    ESP_LOGI(TAG, "Internal free heap: %u bytes", stats.internal_free_heap);
    ESP_LOGI(TAG, "Internal min free heap: %u bytes", stats.internal_min_free_heap);

    uint32_t usage_percent = diagnostics_get_heap_usage_percentage();
    ESP_LOGI(TAG, "Heap usage: %u%%", usage_percent);

    if (usage_percent > 80) {
        ESP_LOGW(TAG, "WARNING: High heap usage detected!");
    }
}

void diagnostics_print_task_stats(void)
{
    const uint32_t max_tasks = 20;
    task_stats_t task_stats[max_tasks];
    uint32_t num_tasks = 0;

    diagnostics_get_task_stats(task_stats, max_tasks, &num_tasks);

    ESP_LOGI(TAG, "=== TASK STATISTICS (%u tasks) ===", num_tasks);
    ESP_LOGI(TAG, "%-16s %-8s %-8s %-12s %-8s",
             "Task Name", "Priority", "State", "Stack Left", "Runtime");
    ESP_LOGI(TAG, "----------------------------------------------------------------");

    for (uint32_t i = 0; i < num_tasks; i++) {
        const char *state_str;
        switch (task_stats[i].state) {
            case eRunning:   state_str = "Running"; break;
            case eReady:     state_str = "Ready"; break;
            case eBlocked:   state_str = "Blocked"; break;
            case eSuspended: state_str = "Suspend"; break;
            case eDeleted:   state_str = "Deleted"; break;
            default:         state_str = "Unknown"; break;
        }

        ESP_LOGI(TAG, "%-16s %-8u %-8s %-12u %-8u",
                 task_stats[i].task_name,
                 (unsigned int)task_stats[i].priority,
                 state_str,
                 (unsigned int)task_stats[i].stack_high_water_mark,
                 (unsigned int)task_stats[i].runtime_counter);
    }
}

void diagnostics_get_heap_stats(heap_stats_t *stats)
{
    if (!stats) return;

    // Get total heap size using heap_caps functions
    stats->total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    stats->free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    stats->min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    stats->largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    stats->internal_free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    stats->internal_min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
}

void diagnostics_get_task_stats(task_stats_t *stats, uint32_t max_tasks, uint32_t *num_tasks)
{
    if (!stats || !num_tasks) return;

    TaskStatus_t *task_status_array = NULL;
    UBaseType_t task_count = uxTaskGetNumberOfTasks();

    if (task_count > max_tasks) {
        task_count = max_tasks;
    }

    task_status_array = pvPortMalloc(task_count * sizeof(TaskStatus_t));
    if (!task_status_array) {
        ESP_LOGE(TAG, "Failed to allocate memory for task stats");
        *num_tasks = 0;
        return;
    }

    task_count = uxTaskGetSystemState(task_status_array, task_count, NULL);

    for (UBaseType_t i = 0; i < task_count; i++) {
        strncpy(stats[i].task_name, task_status_array[i].pcTaskName, sizeof(stats[i].task_name) - 1);
        stats[i].task_name[sizeof(stats[i].task_name) - 1] = '\0';
        stats[i].stack_high_water_mark = task_status_array[i].usStackHighWaterMark;
        stats[i].priority = task_status_array[i].uxCurrentPriority;
        stats[i].state = task_status_array[i].eCurrentState;
        stats[i].runtime_counter = task_status_array[i].ulRunTimeCounter;
    }

    *num_tasks = task_count;
    vPortFree(task_status_array);
}

uint32_t diagnostics_get_heap_usage_percentage(void)
{
    uint32_t total = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    uint32_t free = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);

    if (total == 0) return 100;

    uint32_t used = total - free;
    return (used * 100) / total;
}
