#include "spi.h"
#include "spi_mutex.h"
#include <string.h>

spi_device_handle_t spi; 


void spiBegin(void)
{
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,  // DMA-safe: larger buffer for efficient DMA transfers
    };

    spi_device_interface_config_t devcfg={
        .mode = 0,                                // SPI mode 0
        .spics_io_num = -1,                       // CS pin (managed externally)
        .queue_size = 3,                          // DMA-safe: smaller queue for DMA efficiency
        .clock_speed_hz = CAMERA_SPI_CLK_FREQ,
        .flags = SPI_DEVICE_NO_DUMMY,             // DMA-safe: no dummy bits
    };

    // Initialize with DMA channel for thread-safe, efficient transfers
    spi_bus_initialize(CAMERA_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(CAMERA_HOST, &devcfg, &spi);

    // Initialize SPI mutex for camera
    spi_mutex_init(CAMERA_HOST);

}

void spiCsOutputMode(int cs)
{
    gpio_config_t io_conf = {};
    uint64_t pinSel = 1ULL << cs;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = pinSel;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void spiCsHigh(int cs)
{
    gpio_set_level(cs, 1);
}

void spiCsLow(int cs)
{
    gpio_set_level(cs, 0);
}

uint8_t spiReadWriteByte(uint8_t val)
{
    // Acquire SPI mutex for thread-safe access
    spi_mutex_acquire(CAMERA_HOST);

    uint8_t rt = 0;
    spi_transaction_t t;

    // DMA-safe transaction configuration
    memset(&t, 0, sizeof(t));
    t.length = 8;                    // 8 bits to transmit
    t.rxlength = 8;                  // 8 bits to receive
    t.tx_buffer = &val;              // DMA-safe: use buffer instead of tx_data
    t.rx_buffer = &rt;               // DMA-safe: use buffer instead of rx_data
    t.user = (void*)0;               // User context (unused)

    // Use DMA-capable polling transmit
    esp_err_t err = spi_device_polling_transmit(spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE("SPI", "DMA-safe transaction failed: %s", esp_err_to_name(err));
        rt = 0; // Return 0 on error
    }

    // Release SPI mutex
    spi_mutex_release(CAMERA_HOST);

    return rt;
}