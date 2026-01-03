# Matcha-Following Rover

Autonomous rover using ESP32-CAM + Edge Impulse ML to detect and follow a "matcha".

---

## 1. Quick Reference

### Architecture
```
ESP32-CAM                              ESP32 Rover
┌──────────────────┐  Serial 4800     ┌──────────────────┐
│ Edge Impulse ML  │  GPIO14→GPIO16   │ L298N Motors (4) │
│ WiFi Streaming   │ ───────────────> │ VL53L0X ToF      │
│                  │                  │ Lift Servo       │
│ /stream          │                  │ Web Control      │
│ /detection       │                  │                  │
│ /reset           │                  │                  │
└──────────────────┘                  └──────────────────┘
   http://<CAM-IP>                       http://<ROVER-IP>
```

### Components
| Component | Qty | Purpose |
|-----------|-----|---------|
| ESP32-CAM (AI-Thinker) | 1 | Camera + ML inference |
| ESP32 DevKit | 1 | Motor control + sensors |
| L298N Motor Driver | 1 | 2 channels, 4 motors |
| VL53L0X ToF Sensor | 1 | Obstacle detection |
| SG90 Servo | 1 | Lift arm | 
| DC Motors | 4 | 2 left + 2 right (parallel) |

### Files
| File | IDE |
|------|-----|
| `esp32cam/src/main.cpp` | PlatformIO |
| `esp32cam/esp32cam/esp32cam.ino` | Arduino IDE |
| `esp32_dev_board/src/main.cpp` | PlatformIO |
| `esp32_dev_board/esp32_dev_board/esp32_dev_board.ino` | Arduino IDE |

**Note:** PlatformIO projects use `src/main.cpp`. For Arduino IDE, use the `.ino` file in the matching folder name (e.g., `esp32_dev_board/esp32_dev_board/*.ino`).

---

## 2. Wiring

### Power
```
Battery (7-12V)
    │
    ├── (+) ──────────> L298N +12V/VMS (motor power input)
    │
    └── (-) ──────────> L298N GND ───┬───> L298N 5V ───> ESP32 5V
                                     │
                                     └───> ESP32 GND (common ground)
```
**Note:** L298N's onboard regulator outputs 5V when VMS receives 7-12V.

### ESP32 Dev Board Board Pin Layout
```
Each 3-pin header: [G] [V] [S]
                   GND 3.3V Signal(io##)
```

### L298N Motor Driver Connections
| ESP32 Dev Board Header | GPIO | S Pin | Connect To |
|-------------------|------|-------|------------|
| io13 | GPIO13 | S | L298N IN1 (Left FWD) |
| io14 | GPIO14 | S | L298N IN2 (Left BWD) |
| io33 | GPIO33 | S | L298N IN3 (Right FWD) |
| io32 | GPIO32 | S | L298N IN4 (Right BWD) |
| (any) | - | V (3.3V) | L298N ENA |
| (any) | - | V (3.3V) | L298N ENB |
| (bottom row) | - | GND | L298N GND |
| (bottom row) | - | 5V | L298N 5V output |

**Note:** GPIO26 and GPIO27 are broken on this board (stuck HIGH) - do not use.

### Servo Connections (use 3-pin headers directly)
| Servo | ESP32 Dev Board Header | GPIO | G | V | S |
|-------|-------------------|------|---|---|---|
| Lift Servo | io25 | GPIO25 | GND (brown) | 3.3V (red) | Signal (orange) |

### VL53L0X ToF Sensor (use I2C header)
| ToF Pin | ESP32 Dev Board I2C Header | GPIO |
|---------|----------------------|------|
| GND | GND | - |
| VCC | V (3.3V) | - |
| SDA | SDA | GPIO21 |
| SCL | SCL | GPIO22 |

### ESP32-CAM Connection
| ESP32-CAM | ESP32 Dev Board |
|-----------|------------|
| IO14 (TX) | io16 S pin |
| 5V | 5V (from VIN) |
| GND | GND |

### Wiring Diagram
```
ESP32-CAM              ESP32 Dev Board                     L298N                    Motors
┌───────────┐         ┌─────────────┐                ┌─────────────┐
│    IO14 ──┼────────>│ io16 [S]    │                │             │
│           │         │             │                │             │
│       5V <┼─────────┼─ VIN        │                │             │
│      GND ─┼────────>│ GND         │                │             │
└───────────┘         │             │                │             │
                      │ io13 [S] ───┼───────────────>│ IN1         │──> Left motors
                      │ io14 [S] ───┼───────────────>│ IN2         │    (parallel)
                      │ (any) [V] ──┼───────────────>│ ENA         │
                      │             │                │             │
                      │ io33 [S] ───┼───────────────>│ IN3         │──> Right motors
                      │ io32 [S] ───┼───────────────>│ IN4         │    (parallel)
                      │ (any) [V] ──┼───────────────>│ ENB         │
                      │             │                │             │
                      │ VIN <───────┼────────────────│ 5V          │
                      │ GND ────────┼───────────────>│ GND         │
                      │             │                └─────────────┘
                      │ io25 [G V S]┼──> Lift Servo
                      │ [GND 3.3V GPIO21 GPIO22] ──> ToF (SDA/SCL)
                      └─────────────┘
```

---

## 3. Software

### Commands (ESP32-CAM → Rover)

Format: `COMMAND|label|confidence` (e.g., `SLOW|matcha|85`)

| Command | Action |
|---------|--------|
| `FORWARD` | Both motors forward (continuous) |
| `BACKWARD` | Both motors backward (continuous) |
| `SLOW` | Forward pulse (125ms) |
| `TURN_LEFT` | Spin left (100ms) |
| `TURN_RIGHT` | Spin right (100ms) |
| `SEARCH_LEFT` | Spin left, wall-aware |
| `SEARCH_RIGHT` | Spin right, wall-aware |
| `GOAL` | Stop - goal reached |
| `STOP` | Pause - stop motors, ignore CAM commands |
| `RESUME` | Resume listening to CAM commands |
| `LIFT_UP` | Raise arm (90°) |
| `LIFT_DOWN` | Lower arm (0°) |

### ESP32-CAM Endpoints

| Endpoint | Response |
|----------|----------|
| `/` | Web viewer with detection overlay |
| `/stream` | MJPEG video stream |
| `/detection` | JSON: `{detected, label, confidence, bbox, offset, command}` |
| `/reset` | Reset goal state, resume search |

### Navigation Logic

```
if goalReached           → GOAL (stop)
else if no detection ×3  → SEARCH_LEFT or SEARCH_RIGHT
else if matcha detected  → SLOW (approach)
```

**Goal Detection:** Reached when `centeredCount >= 3` AND `slowCount >= 10`

**ToF Obstacle:**
- Tracking mode: Stop at <10cm (target reached)
- Search mode: Stop at <20cm (wall avoidance)

### Tuning Parameters

| Parameter | Default | File |
|-----------|---------|------|
| `MIN_CONFIDENCE` | 0.50 | esp32cam/src/main.cpp |
| `CENTERED_THRESHOLD` | 3 | esp32cam/src/main.cpp |
| `SLOW_THRESHOLD` | 10 | esp32cam/src/main.cpp |
| `OBSTACLE_DISTANCE_CM` | 20 | esp32_dev_board/src/main.cpp |
| `TARGET_DISTANCE_CM` | 10 | esp32_dev_board/src/main.cpp |
| `PULSE_DURATION` | 100ms | esp32_dev_board/src/main.cpp |
| `SLOW_PULSE_DURATION` | 125ms | esp32_dev_board/src/main.cpp |
| `STEER_PULSE_DURATION` | 75ms | esp32_dev_board/src/main.cpp |
| `SERVO_CENTER` | 70 | esp32_dev_board/src/main.cpp |

---

## 4. Setup & Troubleshooting

### Setup Steps

**1. ESP32-CAM (Arduino IDE)**
- Board: AI Thinker ESP32-CAM
- Partition: Huge APP (3MB No OTA)
- Update WiFi credentials → Upload

**2. ESP32 Rover (PlatformIO)**
- Update WiFi in `src/main.cpp`
- Run: `pio run -t upload`

**3. Wiring**
- ESP32-CAM GPIO14 → Rover GPIO16
- L298N: IN1→GPIO13, IN2→GPIO14, IN3→GPIO33, IN4→GPIO32
- ToF: SDA→GPIO21, SCL→GPIO22
- Lift Servo: GPIO25

**4. Test**
- Check Serial Monitors for IPs
- Open `http://<CAM-IP>/` for detection view
- Open `http://<ROVER-IP>/` for manual control

### Troubleshooting

| Problem | Fix |
|---------|-----|
| No serial comms | Check GPIO14→GPIO16, baud 4800 |
| Motors don't move | Check L298N wiring, motor power |
| Motors spin continuously | GPIO stuck HIGH - avoid GPIO26/27 |
| Always searching | Improve lighting, lower MIN_CONFIDENCE |
| Goal too early | Increase SLOW_THRESHOLD |
| Hits walls | Check ToF I2C wiring (GPIO21/22) |
| No WiFi | Check credentials, use 2.4GHz |
| Robot too fast | Reduce PULSE_DURATION values |
