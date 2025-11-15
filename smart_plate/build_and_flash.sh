#!/bin/bash

# SmartPlate - Build and Flash Script
# This script builds the ESP32 project and flashes it to the device

echo "=== SmartPlate Build and Flash Script ==="
echo "Building project..."

# Build the project
idf.py build

if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi

echo "✅ Build successful!"
echo "Flashing to device..."

# Flash the project
idf.py flash

if [ $? -ne 0 ]; then
    echo "❌ Flash failed!"
    exit 1
fi

echo "✅ Flash successful!"
echo "Starting serial monitor..."
echo ""
echo "Expected output:"
echo "- SmartPlate System Ready!"
echo "- Initializing UI screens..."
echo "- Smart Plate UI initialized"
echo ""
echo "⚠️  IMPORTANT: Ensure WiFi is configured and API server is running"
echo "   API URL: Check API_BASE_URL in main/smart_plate_main.c"
echo ""
echo "Press Ctrl+C to exit monitor"

# Start monitoring
idf.py monitor
