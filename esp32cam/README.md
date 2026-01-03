# ESP32-CAM Object Detection

Edge Impulse ML-powered object detection for ESP32-CAM.

## Which File to Use

| IDE | File |
|-----|------|
| **Arduino IDE** | `esp32cam/esp32cam.ino` |
| **PlatformIO** | `src/main.cpp` |

## Arduino IDE Setup

1. Open `esp32cam/esp32cam.ino`
2. Board: **AI Thinker ESP32-CAM**
3. Partition Scheme: **Huge APP (3MB No OTA/1MB SPIFFS)**
4. Install your Edge Impulse library:
   - Sketch → Include Library → Add .ZIP Library
   - Select your exported library: `ei-<your-project>-arduino-x.x.x.zip`

## PlatformIO Setup

1. Open this folder in VS Code with PlatformIO
2. Build and upload: `pio run -t upload`

## Configuration

Edit WiFi credentials in the file you're using:
```cpp
const char* WIFI_SSID = "your-wifi";
const char* WIFI_PASSWORD = "your-password";
```

## Endpoints

| URL | Description |
|-----|-------------|
| `http://<IP>/` | Detection viewer with overlay |
| `http://<IP>/stream` | MJPEG video stream |
| `http://<IP>/detection` | JSON detection data |
| `http://<IP>/reset` | Reset goal state |


ls /dev/cu.*

  pio run -t upload