#ifndef SPI_MUTEX_H
#define SPI_MUTEX_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "esp_log.h"

/**
 * SPI Mutex System for ESP32
 *
 * Provides thread-safe access to SPI buses when multiple devices
 * share the same SPI host. Prevents data corruption and timing issues
 * in multi-tasking environments.
 *
 * Usage:
 * 1. Initialize SPI mutex for each host: spi_mutex_init(host)
 * 2. Acquire mutex before SPI operations: spi_mutex_acquire(host)
 * 3. Perform SPI operations
 * 4. Release mutex after operations: spi_mutex_release(host)
 *
 * Example:
 *   spi_mutex_acquire(HSPI_HOST);
 *   spi_device_polling_transmit(tft_spi, &t);
 *   spi_mutex_release(HSPI_HOST);
 */

#ifdef __cplusplus
extern "C" {
#endif

// SPI host identifiers (using ESP-IDF's SPI_HOST_MAX)

/**
 * Initialize SPI mutex for a specific SPI host
 * Must be called before using SPI operations on that host
 *
 * @param host SPI host to initialize mutex for
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if host invalid
 */
esp_err_t spi_mutex_init(spi_host_device_t host);

/**
 * Acquire SPI mutex for exclusive bus access
 * Blocks until mutex is available
 *
 * @param host SPI host to acquire mutex for
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if host invalid
 */
esp_err_t spi_mutex_acquire(spi_host_device_t host);

/**
 * Release SPI mutex after SPI operations complete
 *
 * @param host SPI host to release mutex for
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if host invalid
 */
esp_err_t spi_mutex_release(spi_host_device_t host);

/**
 * Try to acquire SPI mutex without blocking
 *
 * @param host SPI host to acquire mutex for
 * @return ESP_OK if acquired, ESP_ERR_TIMEOUT if busy, ESP_ERR_INVALID_ARG if host invalid
 */
esp_err_t spi_mutex_try_acquire(spi_host_device_t host);

/**
 * Get current mutex holder task name for debugging
 *
 * @param host SPI host to check
 * @return Task name string or NULL if not held
 */
const char* spi_mutex_get_holder(spi_host_device_t host);

/**
 * Execute DMA-safe SPI transaction with automatic mutex handling
 * This function ensures proper DMA alignment and mutex synchronization
 *
 * @param handle SPI device handle
 * @param host SPI host for mutex
 * @param trans SPI transaction to execute
 * @return ESP_OK on success
 */
esp_err_t spi_mutex_transmit(spi_device_handle_t handle, spi_host_device_t host, spi_transaction_t* trans);

/**
 * Check if a memory buffer is DMA-safe (properly aligned)
 * DMA requires 32-bit alignment for optimal performance
 *
 * @param buffer Memory buffer to check
 * @return true if DMA-safe, false otherwise
 */
bool spi_is_buffer_dma_safe(const void* buffer);

/**
 * Allocate DMA-safe buffer for SPI transactions
 * Uses proper heap capabilities for DMA compatibility
 *
 * @param size Size in bytes
 * @return DMA-safe buffer pointer or NULL on failure
 */
void* spi_dma_malloc(size_t size);

/**
 * Free DMA-safe buffer
 *
 * @param buffer Buffer allocated with spi_dma_malloc
 */
void spi_dma_free(void* buffer);

#ifdef __cplusplus
}
#endif

#endif // SPI_MUTEX_H
