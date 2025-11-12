# ESP32 Arducam Mega Camera Example

This ESP-IDF project demonstrates how to use the Arducam Mega camera module with ESP32, featuring automatic image capture and Base64 encoding for easy viewing.

## Features

- Automatic image capture every 5 seconds
- JPEG image encoding
- Base64 encoding for serial transmission
- UART communication for debugging and data output
- SPI interface for camera communication

## Hardware Requirements

* ESP32 development board (e.g., ESP32-DevKitC, ESP-WROVER-KIT)
* Arducam Mega camera module
* USB cable for power supply and programming
* Jumper wires for SPI connections

## Pin Configuration

| ESP32 Pin | Arducam Mega Pin | Function |
|-----------|------------------|----------|
| GPIO 19   | MISO            | SPI Data In |
| GPIO 26   | MOSI            | SPI Data Out |
| GPIO 25   | SCK             | SPI Clock |
| GPIO 33   | CS              | SPI Chip Select |

## How to Build and Run

### Prerequisites

1. Install ESP-IDF (see [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html))
2. Set up the ESP-IDF environment

### Build and Flash

1. Set the target chip:
   ```bash
   idf.py set-target esp32
   ```

2. Build the project:
   ```bash
   idf.py build
   ```

3. Flash to your ESP32 board:
   ```bash
   idf.py -p <PORT> flash
   ```

4. Monitor the serial output:
   ```bash
   idf.py -p <PORT> monitor
   ```

   (To exit the serial monitor, type `Ctrl-]`)

## Usage

Once flashed and running, the ESP32 will:

1. Initialize the camera and UART communication
2. Automatically capture images every 5 seconds
3. Encode captured JPEG images to Base64 format
4. Send Base64 data through serial port in chunks

## Viewing Images

The captured images are sent as Base64-encoded strings through the serial monitor. To view the images:

1. Copy the Base64 string between "=== JPEG CAPTURE COMPLETE ===" and "=== END OF IMAGE ==="
2. Use an online Base64 to image decoder (e.g., base64.guru)
3. Or use a simple Python script to decode and save the image:

```python
import base64

# Replace with your Base64 string
base64_string = "YOUR_BASE64_STRING_HERE"

# Decode and save as JPEG
with open('captured_image.jpg', 'wb') as f:
    f.write(base64.b64decode(base64_string))
```

## Serial Output Format

```
I (324) main_task: Hello esp32s!
Mega start!
Auto capture mode started!
Taking picture...
Picture taken successfully!
=== JPEG CAPTURE COMPLETE ===
Image size: 12345 bytes
Base64 encoded image:
[Base64 data in chunks]
=== END OF IMAGE ===
```

## Troubleshooting

### Camera Connection Issues
- Verify SPI pin connections match the pin configuration above
- Check camera power supply (3.3V)
- Ensure camera module is properly seated

### Build Errors
- Make sure ESP-IDF is properly installed and sourced
- Check that all required components are present
- Verify the target chip is set correctly (`idf.py set-target esp32`)

### Serial Monitor Issues
- Use 115200 baud rate
- Ensure the correct COM port is selected
- Check USB driver installation

## Project Structure

```
camera/
├── CMakeLists.txt          # Main project CMakeLists
├── sdkconfig              # ESP-IDF configuration
├── main/
│   ├── CMakeLists.txt     # Main component CMakeLists
│   └── app_main.c         # Main application logic
└── components/
    ├── libcamera/         # Camera SPI interface component
    │   ├── CMakeLists.txt
    │   ├── spi.h/.c       # SPI communication
    │   └── Arducam/       # Arducam library files
    └── uart/              # UART communication component
        ├── CMakeLists.txt
        ├── uart.h/.c      # UART functions
        └── ArducamLink.*  # UART link protocol
```

## Technical Details

- **SPI Clock**: 8MHz (conservative for stability)
- **UART Baud Rate**: 115200
- **Image Format**: JPEG (VGA resolution)
- **Capture Interval**: 5 seconds
- **Memory Usage**: ~50KB heap for image buffer

## License

This project uses components from the Arducam Mega library. Please refer to the original Arducam repository for licensing information.
