#ifndef PRIORITIES_H
#define PRIORITIES_H

/**
 * Standardized FreeRTOS Task Priorities for ESP32 Camera Project
 *
 * Priority Levels (Higher numbers = higher priority):
 * - 25: Maximum (configMAX_PRIORITIES - 1)
 * - 0: Minimum (tskIDLE_PRIORITY)
 *
 * Priority Assignment Rationale:
 * ===================
 * CRITICAL (24): System stability, watchdog, critical error handling
 *    - Prevents system crashes, handles fatal errors
 *
 * ISR (22): Interrupt service routines, real-time responses
 *    - GPIO interrupts, timer callbacks, hardware events
 *
 * HIGH (18): Time-critical operations
 *    - Camera capture, UART communications, sensor readings
 *    - Operations that must complete within strict time limits
 *
 * UI (15): User interface tasks
 *    - LVGL rendering, touch input processing, display updates
 *    - Interactive elements that affect user experience
 *
 * NETWORK (12): Communications
 *    - HTTP server, WiFi management, MQTT client
 *    - Network protocols and data transmission
 *
 * BACKGROUND (8): Data processing
 *    - Camera image analysis, data logging, background tasks
 *    - Non-time-critical processing work
 *
 * LOW (5): Maintenance tasks
 *    - Diagnostics, heap monitoring, system health checks
 *    - Periodic maintenance operations
 *
 * IDLE (0): System idle task
 *    - FreeRTOS idle task (don't use directly)
 */

#define TASK_PRIORITY_CRITICAL      24    // System stability, watchdog, critical errors
#define TASK_PRIORITY_ISR           22    // ISR handlers, real-time responses
#define TASK_PRIORITY_HIGH          18    // Time-critical operations (UART, camera)
#define TASK_PRIORITY_UI            15    // User interface (LVGL, touch)
#define TASK_PRIORITY_NETWORK       12    // HTTP server, WiFi
#define TASK_PRIORITY_BACKGROUND    8     // Camera processing, analysis
#define TASK_PRIORITY_LOW           5     // Maintenance, diagnostics
#define TASK_PRIORITY_IDLE          0     // System idle (don't use directly)

#endif // PRIORITIES_H
