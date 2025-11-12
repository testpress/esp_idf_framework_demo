
#include "ArducamCamera.h"
#include "ArducamLink.h"
#include "delay.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

ArducamCamera myCAM;
const int cs = 33;

uint8_t temp             = 0xff;
uint8_t commandBuff[20]  = {0};
uint8_t commandLength    = 0;
uint8_t sendFlag         = TRUE;
uint32_t readImageLength = 0;
uint8_t jpegHeadFlag     = 0;

// JPEG data buffer for base64 encoding
#define MAX_JPEG_SIZE 100000  // 100KB should be enough for VGA JPEG
uint8_t jpegBuffer[MAX_JPEG_SIZE];
uint32_t jpegBufferIndex = 0;

// Base64 encoding table
const char base64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void base64_encode(const uint8_t* data, size_t input_length, char* output) {
    size_t output_index = 0;

    for (size_t i = 0; i < input_length; i += 3) {
        uint32_t octet_a = i < input_length ? data[i] : 0;
        uint32_t octet_b = i + 1 < input_length ? data[i + 1] : 0;
        uint32_t octet_c = i + 2 < input_length ? data[i + 2] : 0;

        uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        output[output_index++] = base64_table[(triple >> 18) & 0x3F];
        output[output_index++] = base64_table[(triple >> 12) & 0x3F];
        output[output_index++] = base64_table[(triple >> 6) & 0x3F];
        output[output_index++] = base64_table[triple & 0x3F];
    }

    // Add padding
    size_t padding_start = ((input_length + 2) / 3) * 4;
    if (input_length % 3 == 1) {
        output[padding_start - 2] = '=';
        output[padding_start - 1] = '=';
    } else if (input_length % 3 == 2) {
        output[padding_start - 1] = '=';
    }

    output[output_index] = '\0';
}

uint8_t ReadBuffer(uint8_t* imagebuf, uint8_t length)
{
    // This function is kept for compatibility but main image processing
    // now happens directly in the main loop
    return sendFlag;
}

void stop_preivew()
{
    readImageLength = 0;
    jpegHeadFlag    = 0;
    jpegBufferIndex = 0;
    uint32_t len    = 9;

    arducamUartWrite(0xff);
    arducamUartWrite(0xBB);
    arducamUartWrite(0xff);
    arducamUartWrite(0xAA);
    arducamUartWrite(0x06);
    arducamUartWriteBuff((uint8_t*)&len, 4);
    printf("streamoff");
    arducamUartWrite(0xff);
    arducamUartWrite(0xBB);
}

void app_main(void)
{
    uartBegin(115200);
    send_data_pack(7,"Hello esp32s!");
    myCAM = createArducamCamera(cs);
    begin(&myCAM);
    send_data_pack(8,"Mega start!");

    registerCallback(&myCAM, ReadBuffer, 200, stop_preivew);

    // Automatically capture images every 5 seconds
    uint32_t capture_counter = 0;
    const uint32_t CAPTURE_INTERVAL = 5000 / portTICK_PERIOD_MS; // 5 seconds

    send_data_pack(9, "Auto capture mode started!");

    while (1) {
        capture_counter++;

        // Take a picture every 5 seconds
        if (capture_counter >= CAPTURE_INTERVAL) {
            capture_counter = 0;

            send_data_pack(10, "Taking picture...");

            // Take VGA JPEG picture (resolution 0x02, format JPEG 0x10)
            CamStatus status = takePicture(&myCAM, CAM_IMAGE_MODE_VGA, CAM_IMAGE_PIX_FMT_JPG);

            if (status == CAM_ERR_SUCCESS) {
                send_data_pack(11, "Picture taken successfully!");

                // Read and encode the image data directly
                uint32_t image_length = imageAvailable(&myCAM);
                if (image_length > 0 && image_length <= MAX_JPEG_SIZE) {
                    printf("Reading image data: %lu bytes\n", image_length);

                    // Read all image data at once
                    uint8_t* image_data = (uint8_t*)malloc(image_length);
                    if (image_data) {
                        uint32_t bytes_read = 0;
                        uint32_t chunk_size = 200; // Read in chunks
                        uint8_t buffer[200];

                        while (bytes_read < image_length) {
                            uint32_t remaining = image_length - bytes_read;
                            uint32_t read_size = (remaining > chunk_size) ? chunk_size : remaining;

                            uint8_t bytes_in_chunk = readBuff(&myCAM, buffer, read_size);
                            if (bytes_in_chunk > 0) {
                                memcpy(&image_data[bytes_read], buffer, bytes_in_chunk);
                                bytes_read += bytes_in_chunk;
                            } else {
                                printf("Failed to read image data chunk\n");
                                break;
                            }
                        }

                        if (bytes_read == image_length) {
                            // Encode to base64 and print
                            printf("=== JPEG CAPTURE COMPLETE ===\n");
                            printf("Image size: %lu bytes\n", image_length);
                            printf("Base64 encoded image:\n");

                            // Calculate base64 buffer size
                            size_t base64_size = ((image_length + 2) / 3) * 4 + 1;
                            char* base64_output = (char*)malloc(base64_size);

                            if (base64_output) {
                                base64_encode(image_data, image_length, base64_output);

                                // Send base64 data in chunks
                                const int CHUNK_SIZE = 64;
                                char* ptr = base64_output;

                                while (*ptr) {
                                    char chunk[CHUNK_SIZE + 1];
                                    int chunk_len = 0;

                                    while (*ptr && chunk_len < CHUNK_SIZE) {
                                        chunk[chunk_len++] = *ptr++;
                                    }
                                    chunk[chunk_len] = '\0';

                                    printf("%s\n", chunk);
                                    vTaskDelay(10 / portTICK_PERIOD_MS);
                                }

                                free(base64_output);
                            } else {
                                printf("ERROR: Failed to allocate base64 buffer!\n");
                            }

                            printf("=== END OF IMAGE ===\n\n");
                        } else {
                            printf("ERROR: Incomplete image data read\n");
                        }

                        free(image_data);
                    } else {
                        printf("ERROR: Failed to allocate image buffer!\n");
                    }
                } else {
                    if (image_length > MAX_JPEG_SIZE) {
                        printf("Image too large: %lu bytes (max: %d)\n", image_length, MAX_JPEG_SIZE);
                    } else {
                        printf("No image data available\n");
                    }
                }
            } else {
                char error_msg[50];
                sprintf(error_msg, "Picture failed with status: %d", status);
                send_data_pack(12, error_msg);
            }
        }

        captureThread(&myCAM);
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
}
