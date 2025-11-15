#include "ArducamLink.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

// SPI handle (global like working camera)
static spi_device_handle_t spi_handle = NULL;
uint8_t current_cs_pin = 0;  // Make this global so ArducamCamera.c can access it

// Camera SPI pin configuration (VSPI_HOST)
// GPIO 19: MISO, GPIO 26: MOSI, GPIO 25: SCLK, GPIO 33: CS
// These pins are separate from display SPI to avoid conflicts

// Initialize SPI (like working camera)
void spiBegin(uint8_t cs_pin) {
    current_cs_pin = cs_pin;

    // Configure CS pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << cs_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(cs_pin, 1);

    // Initialize SPI bus (like working camera)
    spi_bus_config_t buscfg = {
        .miso_io_num = 19,  // MISO pin
        .mosi_io_num = 26,  // MOSI pin
        .sclk_io_num = 25,  // SCLK pin
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    spi_device_interface_config_t devcfg = {
        .mode = 0,                                //SPI mode 0
        .spics_io_num = -1,               //CS pin handled manually
        .queue_size = 256,                          //We want to be able to queue 7 transactions at a time
        .clock_speed_hz = 8 * 1000 * 1000,  // 8 MHz
    };

    spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_DISABLED);
    spi_bus_add_device(VSPI_HOST, &devcfg, &spi_handle);
}

void spiEnd() {
    if (spi_handle) {
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
    }
    spi_bus_free(VSPI_HOST);
}

// SPI byte operations (match working camera)
uint8_t spiReadByte() {
    uint8_t data = 0;
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &data,  // Send the data
        .rxlength = 8,
        .flags = SPI_TRANS_USE_RXDATA
    };
    spi_device_polling_transmit(spi_handle, &trans);
    return trans.rx_data[0];
}

void spiWriteByte(uint8_t data) {
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &data
    };
    spi_device_polling_transmit(spi_handle, &trans);
}

// SPI buffer operations
void spiWriteBuffer(uint8_t *buffer, uint32_t length) {
    spi_transaction_t trans = {
        .length = length * 8,
        .tx_buffer = buffer
    };
    spi_device_polling_transmit(spi_handle, &trans);
}

void spiReadBuffer(uint8_t *buffer, uint32_t length) {
    spi_transaction_t trans = {
        .length = length * 8,
        .rx_buffer = buffer,
        .flags = SPI_TRANS_USE_RXDATA
    };
    spi_device_polling_transmit(spi_handle, &trans);
    memcpy(buffer, trans.rx_data, length);
}

// Delay functions
void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

// UART functions
void uartBegin(uint32_t baud) {
    uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void uartWrite(uint8_t data) {
    uart_write_bytes(UART_NUM_0, (const char*)&data, 1);
}

uint8_t uartRead() {
    uint8_t data = 0;
    uart_read_bytes(UART_NUM_0, &data, 1, 0);
    return data;
}

bool uartAvailable() {
    size_t available = 0;
    uart_get_buffered_data_len(UART_NUM_0, &available);
    return available > 0;
}
