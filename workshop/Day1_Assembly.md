# Day 1: Building Your Robot

Welcome! Today you'll build an autonomous rover that can see and follow objects using artificial intelligence.

---

## What Are We Building?

A robot that can:
- **See** using a camera and AI (machine learning)
- **Think** by recognizing objects you train it to find
- **Move** using 4 motors controlled by a motor driver
- **Sense** obstacles using a laser distance sensor

```
┌─────────────┐     commands      ┌─────────────┐
│  ESP32-CAM  │  ──────────────>  │ ESP32 Rover │
│  (Brain #1) │                   │  (Brain #2) │
│  - Camera   │                   │  - Motors   │
│  - AI Model │                   │  - Sensors  │
└─────────────┘                   └─────────────┘
```

---

## Meet Your Components

### 1. ESP32-CAM (The Eyes + AI Brain)
A tiny computer with a built-in camera. It runs the AI model that recognizes objects.

**What it does:** Takes pictures, runs AI to detect objects, sends movement commands.

### 2. ESP32 Dev Board (The Motor Brain)
Another tiny computer that controls all the physical parts - motors, sensors, and servo.

**What it does:** Receives commands from ESP32-CAM, controls motors, reads sensors.

### 3. L298N Motor Driver (The Muscle)
Motors need more power than the ESP32 can provide. The L298N is like a power amplifier for motors.

**What it does:** Takes small signals from ESP32 and converts them to powerful motor signals.

**Key parts:**
- **IN1, IN2, IN3, IN4** - Control inputs (connect to ESP32)
- **OUT1, OUT2, OUT3, OUT4** - Motor outputs
- **ENA, ENB** - Enable pins (must be HIGH for motors to work)
- **+12V** - Motor power input (7-12V battery)
- **5V** - Power output to ESP32
- **GND** - Ground (must be shared with ESP32)

### 4. DC Motors (4x)
Small motors that spin the wheels. We use 4 motors - 2 on the left, 2 on the right.

**Left motors:** Connected in parallel to OUT1/OUT2
**Right motors:** Connected in parallel to OUT3/OUT4

### 5. VL53L0X ToF Sensor (The Distance Eyes)
"ToF" stands for "Time of Flight". It shoots a tiny laser beam and measures how long it takes to bounce back.

**What it does:** Measures distance to obstacles (up to 2 meters).

### 6. SG90 Servo (The Arm)
A motor that can rotate to specific angles (0° to 180°).

**What it does:** Lifts an arm to pick up objects (optional feature).

---

## Tools You'll Need

- Small Phillips screwdriver
- Wire strippers (if using raw wire)
- Dupont jumper wires (male-to-female, female-to-female)
- USB cables (2x) - for programming both ESP32 boards

---

## Step 1: Understand the ESP32 Dev Board Pins

Your ESP32 Dev Board has rows of 3-pin headers:
```
Each header: [G] [V] [S]
              │   │   └── Signal (GPIO pin)
              │   └────── Voltage (3.3V)
              └────────── Ground (GND)
```

The headers are labeled: io13, io14, io16, io25, io32, io33, etc.

---

## Step 2: Wire the Power

**IMPORTANT:** Double-check all power connections before connecting the battery!

### Power Flow:
```
Battery (7-12V)
    │
    ├── (+) ────> L298N "+12V" terminal
    │
    └── (-) ────> L298N "GND" terminal
                      │
                      └──> L298N outputs 5V ──> ESP32 "VIN" pin
```

### Connections:
| From | To |
|------|-----|
| Battery (+) | L298N +12V terminal |
| Battery (-) | L298N GND terminal |
| L298N 5V output | ESP32 Dev Board VIN |
| L298N GND | ESP32 Dev Board GND |

**Why this works:** The L298N has a built-in voltage regulator that converts 7-12V to 5V.

---

## Step 3: Wire the Motors

### Motor Channel A (Left Motors)
Both left motors connect in parallel to the same outputs:

| L298N Terminal | Connect To |
|---------------|------------|
| OUT1 | Left Motor 1 (+) AND Left Motor 2 (+) |
| OUT2 | Left Motor 1 (-) AND Left Motor 2 (-) |

### Motor Channel B (Right Motors)
Both right motors connect in parallel:

| L298N Terminal | Connect To |
|---------------|------------|
| OUT3 | Right Motor 1 (+) AND Right Motor 2 (+) |
| OUT4 | Right Motor 1 (-) AND Right Motor 2 (-) |

**Tip:** If a motor spins the wrong way later, just swap its two wires.

---

## Step 4: Wire the Control Pins

Now connect the ESP32 to the L298N control inputs:

| ESP32 Header | GPIO | L298N Pin | Purpose |
|-------------|------|-----------|---------|
| io13 | GPIO13 | IN1 | Left motors forward |
| io14 | GPIO14 | IN2 | Left motors backward |
| io33 | GPIO33 | IN3 | Right motors forward |
| io32 | GPIO32 | IN4 | Right motors backward |
| (any) | V pin | ENA | Enable left motors |
| (any) | V pin | ENB | Enable right motors |

**How to connect:**
1. Take a jumper wire
2. Connect the **S** (signal) pin of io13 header to L298N **IN1**
3. Repeat for io14→IN2, io33→IN3, io32→IN4

**Enable pins:** Connect ENA and ENB to any **V** (3.3V) pin. This keeps motors enabled.

---

## Step 5: Wire the ToF Distance Sensor

The VL53L0X uses I2C communication (2 wires for data).

| ToF Sensor Pin | ESP32 Pin |
|---------------|-----------|
| VCC | 3.3V (any V pin) |
| GND | GND (any G pin) |
| SDA | GPIO21 (I2C header) |
| SCL | GPIO22 (I2C header) |

**Tip:** Your ESP32 Dev Board should have a dedicated I2C header with SDA/SCL labeled.

---

## Step 6: Wire the Servo

The SG90 servo has 3 wires:
- **Brown** = Ground
- **Red** = Power (5V or 3.3V)
- **Orange** = Signal

| Servo Wire | ESP32 Header io25 |
|-----------|-------------------|
| Brown (GND) | G pin |
| Red (Power) | V pin |
| Orange (Signal) | S pin |

---

## Step 7: Wire the ESP32-CAM Connection

The ESP32-CAM sends commands to the ESP32 Dev Board via a serial connection.

| ESP32-CAM Pin | ESP32 Dev Board |
|--------------|-----------------|
| GPIO14 (TX) | io16 S pin |
| 5V | VIN |
| GND | GND |

**Important:**
- ESP32-CAM GPIO14 sends data OUT (TX)
- ESP32 Dev Board GPIO16 receives data IN (RX)

---

## Complete Wiring Diagram

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
                      │ io25 [G V S]┼──> Lift Servo (brown/red/orange)
                      │             │
                      │ I2C Header ─┼──> ToF Sensor (VCC/GND/SDA/SCL)
                      └─────────────┘
```

---

## Step 8: Verification Checklist

Before powering on, check each connection:

### Power
- [ ] Battery (+) connected to L298N +12V
- [ ] Battery (-) connected to L298N GND
- [ ] L298N 5V connected to ESP32 VIN
- [ ] L298N GND connected to ESP32 GND
- [ ] ESP32-CAM getting power from ESP32

### Motors
- [ ] Left motors connected to OUT1/OUT2
- [ ] Right motors connected to OUT3/OUT4
- [ ] IN1 connected to GPIO13
- [ ] IN2 connected to GPIO14
- [ ] IN3 connected to GPIO33
- [ ] IN4 connected to GPIO32
- [ ] ENA connected to 3.3V
- [ ] ENB connected to 3.3V

### Sensors
- [ ] ToF VCC to 3.3V
- [ ] ToF GND to GND
- [ ] ToF SDA to GPIO21
- [ ] ToF SCL to GPIO22

### Servo
- [ ] Servo brown wire to GND
- [ ] Servo red wire to 3.3V
- [ ] Servo orange wire to GPIO25

### Communication
- [ ] ESP32-CAM GPIO14 to ESP32 Dev Board GPIO16

---

## Troubleshooting

| Problem | Check |
|---------|-------|
| Nothing powers on | Battery connections, polarity (+/-) |
| Motors don't spin | ENA/ENB connected to 3.3V? |
| One motor spins wrong way | Swap its two wires at OUT terminals |
| ESP32-CAM doesn't power | Check 5V and GND connections |

---

## What's Next?

Tomorrow (Day 2), you'll:
1. Upload code to both ESP32 boards
2. Capture training images of your object
3. Train an AI model on Edge Impulse
4. Deploy the model and test your rover!

---

## Glossary

| Term | Meaning |
|------|---------|
| GPIO | General Purpose Input/Output - pins that can send or receive signals |
| I2C | A communication protocol using 2 wires (SDA for data, SCL for clock) |
| Serial | Communication by sending data one bit at a time |
| PWM | Pulse Width Modulation - a way to control motor speed |
| GND | Ground - the 0V reference point, must be shared between all devices |
| VIN | Voltage Input - where you connect external power |
