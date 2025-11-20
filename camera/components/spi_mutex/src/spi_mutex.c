#include "spi_mutex.h"
#include "esp_heap_caps.h"
#include <string.h>

#define TAG "spi_mutex"

// SPI mutex structure for each host
typedef struct {
    SemaphoreHandle_t mutex;
    const char* holder_task_name;
    bool initialized;
} spi_mutex_t;

// Array of mutexes for all SPI hosts (SPI_HOST_MAX is defined by ESP-IDF)
static spi_mutex_t spi_mutexes[SPI_HOST_MAX] = {0};

/**
 * Convert SPI host device to array index
 */
static esp_err_t spi_host_to_index(spi_host_device_t host, int* index) {
    switch (host) {
        case SPI1_HOST:
            *index = 0;
            break;
        case SPI2_HOST:
            *index = 1;
            break;
        case SPI3_HOST:
            *index = 2;
            break;
        default:
            ESP_LOGE(TAG, "Invalid SPI host: %d", host);
            return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

/**
 * Initialize SPI mutex for a specific SPI host
 */
esp_err_t spi_mutex_init(spi_host_device_t host) {
    int index;
    esp_err_t ret = spi_host_to_index(host, &index);

    if (ret != ESP_OK) {
        return ret;
    }

    if (spi_mutexes[index].initialized) {
        ESP_LOGW(TAG, "SPI mutex for host %d already initialized", host);
        return ESP_OK;
    }

    // Create mutex
    spi_mutexes[index].mutex = xSemaphoreCreateMutex();
    if (spi_mutexes[index].mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create SPI mutex for host %d", host);
        return ESP_ERR_NO_MEM;
    }

    spi_mutexes[index].holder_task_name = NULL;
    spi_mutexes[index].initialized = true;

    ESP_LOGI(TAG, "SPI mutex initialized for host %d", host);
    return ESP_OK;
}

/**
 * Acquire SPI mutex for exclusive bus access
 */
esp_err_t spi_mutex_acquire(spi_host_device_t host) {
    int index;
    esp_err_t ret = spi_host_to_index(host, &index);

    if (ret != ESP_OK) {
        return ret;
    }

    if (!spi_mutexes[index].initialized) {
        ESP_LOGE(TAG, "SPI mutex for host %d not initialized", host);
        return ESP_ERR_INVALID_STATE;
    }

    // Acquire mutex (blocks until available)
    if (xSemaphoreTake(spi_mutexes[index].mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire SPI mutex for host %d", host);
        return ESP_FAIL;
    }

    // Store current task name for debugging
    spi_mutexes[index].holder_task_name = pcTaskGetName(NULL);

    ESP_LOGD(TAG, "SPI mutex acquired for host %d by task '%s'",
             host, spi_mutexes[index].holder_task_name);

    return ESP_OK;
}

/**
 * Release SPI mutex after SPI operations complete
 */
esp_err_t spi_mutex_release(spi_host_device_t host) {
    int index;
    esp_err_t ret = spi_host_to_index(host, &index);

    if (ret != ESP_OK) {
        return ret;
    }

    if (!spi_mutexes[index].initialized) {
        ESP_LOGE(TAG, "SPI mutex for host %d not initialized", host);
        return ESP_ERR_INVALID_STATE;
    }

    const char* holder = spi_mutexes[index].holder_task_name;
    spi_mutexes[index].holder_task_name = NULL;

    if (xSemaphoreGive(spi_mutexes[index].mutex) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to release SPI mutex for host %d", host);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "SPI mutex released for host %d (was held by '%s')", host, holder);
    return ESP_OK;
}

/**
 * Try to acquire SPI mutex without blocking
 */
esp_err_t spi_mutex_try_acquire(spi_host_device_t host) {
    int index;
    esp_err_t ret = spi_host_to_index(host, &index);

    if (ret != ESP_OK) {
        return ret;
    }

    if (!spi_mutexes[index].initialized) {
        ESP_LOGE(TAG, "SPI mutex for host %d not initialized", host);
        return ESP_ERR_INVALID_STATE;
    }

    // Try to acquire mutex without blocking
    if (xSemaphoreTake(spi_mutexes[index].mutex, 0) == pdTRUE) {
        spi_mutexes[index].holder_task_name = pcTaskGetName(NULL);
        ESP_LOGD(TAG, "SPI mutex try-acquired for host %d by task '%s'",
                 host, spi_mutexes[index].holder_task_name);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT; // Mutex is busy
}

/**
 * Get current mutex holder task name for debugging
 */
const char* spi_mutex_get_holder(spi_host_device_t host) {
    int index;
    if (spi_host_to_index(host, &index) != ESP_OK) {
        return NULL;
    }

    if (!spi_mutexes[index].initialized) {
        return NULL;
    }

    return spi_mutexes[index].holder_task_name;
}

/**
 * Execute DMA-safe SPI transaction with automatic mutex handling
 */
esp_err_t spi_mutex_transmit(spi_device_handle_t handle, spi_host_device_t host, spi_transaction_t* trans) {
    esp_err_t ret;

    // Acquire SPI mutex
    ret = spi_mutex_acquire(host);
    if (ret != ESP_OK) {
        return ret;
    }

    // Execute DMA-safe SPI transaction
    ret = spi_device_polling_transmit(handle, trans);

    // Release SPI mutex
    spi_mutex_release(host);

    return ret;
}

/**
 * Check if a memory buffer is DMA-safe (properly aligned)
 */
bool spi_is_buffer_dma_safe(const void* buffer) {
    if (buffer == NULL) {
        return false;
    }

    // DMA requires at least 4-byte alignment for optimal performance
    uintptr_t addr = (uintptr_t)buffer;
    return (addr & 0x3) == 0; // Check 4-byte alignment
}

/**
 * Allocate DMA-safe buffer for SPI transactions
 */
void* spi_dma_malloc(size_t size) {
    // Try PSRAM first (DMA-capable with 8-bit alignment)
    void* buffer = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (buffer == NULL) {
        // Fallback to internal RAM (DMA-capable)
        buffer = heap_caps_malloc(size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    }

    if (buffer != NULL && !spi_is_buffer_dma_safe(buffer)) {
        ESP_LOGW(TAG, "Allocated buffer is not DMA-aligned, this may impact performance");
    }

    return buffer;
}

/**
 * Free DMA-safe buffer
 */
void spi_dma_free(void* buffer) {
    if (buffer != NULL) {
        heap_caps_free(buffer);
    }
}
