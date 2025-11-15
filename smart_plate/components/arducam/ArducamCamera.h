#ifndef ARDUCAM_CAMERA_H
#define ARDUCAM_CAMERA_H

#include <stdint.h>
#include <stdbool.h>

// Camera status enumeration
typedef enum {
    CAM_ERR_SUCCESS = 0,
    CAM_ERR_NO_CALLBACK,
    CAM_ERR_INVALID_COMMAND,
    CAM_ERR_INVALID_SIZE,
    CAM_ERR_FAILURE
} CamStatus;

// Image mode enumeration
typedef enum {
    CAM_IMAGE_MODE_QVGA = 0,
    CAM_IMAGE_MODE_VGA,
    CAM_IMAGE_MODE_SVGA,
    CAM_IMAGE_MODE_HD,
    CAM_IMAGE_MODE_SXGAM,
    CAM_IMAGE_MODE_UXGA,
    CAM_IMAGE_MODE_WQXGA2,
    CAM_IMAGE_MODE_UXGA2,
    CAM_IMAGE_MODE_QQVGA,
    CAM_IMAGE_MODE_CIF,
    CAM_IMAGE_MODE_OCIF,
    CAM_IMAGE_MODE_QCIF,
    CAM_IMAGE_MODE_OQCIF,
    CAM_IMAGE_MODE_QQVGA2,
    CAM_IMAGE_MODE_QVGA2,
    CAM_IMAGE_MODE_CIF2,
    CAM_IMAGE_MODE_OCIF2,
    CAM_IMAGE_MODE_QCIF2,
    CAM_IMAGE_MODE_OQCIF2
} CamImageMode;

// Pixel format enumeration
typedef enum {
    CAM_IMAGE_PIX_FMT_JPG = 0,
    CAM_IMAGE_PIX_FMT_RGB565,
    CAM_IMAGE_PIX_FMT_YUV
} CamImagePixFmt;

// Camera structure
typedef struct {
    uint8_t cs_pin;
    uint8_t spi_bus;
    bool initialized;
    uint32_t image_length; // Store captured image length
} ArducamCamera;

// Function declarations
ArducamCamera createArducamCamera(uint8_t cs_pin);
CamStatus begin(ArducamCamera *cam);
CamStatus takePicture(ArducamCamera *cam, CamImageMode mode, CamImagePixFmt fmt);
uint32_t imageAvailable(ArducamCamera *cam);
uint8_t readBuff(ArducamCamera *cam, uint8_t *buff, uint8_t size);
void captureThread(ArducamCamera *cam);

#endif // ARDUCAM_CAMERA_H
