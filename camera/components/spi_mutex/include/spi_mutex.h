#ifndef SPI_MUTEX_H
#define SPI_MUTEX_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hal/spi_types.h"
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

#ifdef __cplusplus
}
#endif

#endif // SPI_MUTEX_H
