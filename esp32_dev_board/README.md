# ESP32 Dev Board Motor Controller

Receives commands from ESP32-CAM and controls the rover motors.

## Which File to Use

| IDE | File |
|-----|------|
| **Arduino IDE** | `esp32_dev_board/esp32_dev_board.ino` |
| **PlatformIO** | `src/main.cpp` |

## Arduino IDE Setup

1. Open `esp32_dev_board/esp32_dev_board.ino`
2. Board: **ESP32 Dev Module**
3. Partition Scheme: **Default 4MB with spiffs**
4. Install libraries:
   - ESP32Servo
   - VL53L0X (Pololu)

## PlatformIO Setup

1. Open this folder in VS Code with PlatformIO
2. Build and upload: `pio run -t upload`

## Configuration

Edit WiFi credentials:
```cpp
const char* WIFI_SSID = "your-wifi";
const char* WIFI_PASSWORD = "your-password";
```

## Web Interface

Access rover control at: `http://<IP>/`

| Endpoint | Description |
|----------|-------------|
| `/` | Manual control interface |
| `/cmd?action=FORWARD` | Send command |
| `/status` | Get current status |
| `/log` | Command log |
| `/tof` | ToF sensor readings |
