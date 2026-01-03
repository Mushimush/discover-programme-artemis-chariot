# Day 3: Fine-Tuning & Obstacle Avoidance

Today you'll make your robot smarter and more reliable!

---

## Today's Goals

1. **Troubleshoot** any issues from Day 2
2. **Tune parameters** for better performance
3. **Understand** obstacle avoidance
4. **Challenge:** Navigate a course!

---

## Part 1: Common Issues & Fixes

### Issue: Robot Spins Forever (Never Finds Object)

**Possible causes:**
1. Camera can't see the object clearly
2. AI confidence is too low
3. Object looks different from training images

**Fixes:**
- Improve lighting (avoid shadows)
- Hold object closer to camera
- Lower the confidence threshold (see Part 2)
- Retrain with more images

### Issue: Robot Overshoots the Object

**Cause:** Robot moves too fast or too long before checking again.

**Fix:** Reduce pulse durations in code (see Part 2).

### Issue: Robot Moves Wrong Direction

**Cause:** Motor wires swapped.

**Fix:** Swap the two wires for that motor at the L298N.

### Issue: Robot Stops Too Far from Object

**Cause:** ToF sensor distance threshold too high.

**Fix:** Lower `TARGET_DISTANCE_CM` value (see Part 2).

### Issue: False Detections (Sees Object When There Isn't One)

**Cause:** Not enough "background" training images.

**Fix:**
1. Capture more background images
2. Retrain on Edge Impulse
3. Increase confidence threshold

---

## Part 2: Tuning Parameters

You can adjust these values in the code to change how your robot behaves.

### ESP32-CAM Parameters

**File:** `esp32cam/src/main.cpp`

| Parameter | Default | What It Does |
|-----------|---------|--------------|
| `MIN_CONFIDENCE` | 0.50 | Minimum confidence to count as detection (0.0-1.0) |
| `CENTERED_THRESHOLD` | 3 | How many "centered" frames before GOAL |
| `SLOW_THRESHOLD` | 10 | How many SLOW commands before GOAL |
| `NO_DETECTION_THRESHOLD` | 3 | Frames without detection before searching |

**Examples:**
- Robot stops too early? **Increase** SLOW_THRESHOLD
- Robot misses obvious objects? **Lower** MIN_CONFIDENCE to 0.40
- Robot triggers false GOALs? **Increase** CENTERED_THRESHOLD

### ESP32 Dev Board Parameters

**File:** `esp32_dev_board/src/main.cpp`

| Parameter | Default | What It Does |
|-----------|---------|--------------|
| `PULSE_DURATION` | 100ms | How long each turn lasts |
| `SLOW_PULSE_DURATION` | 125ms | How long each forward movement lasts |
| `STEER_PULSE_DURATION` | 75ms | How long steering corrections last |
| `OBSTACLE_DISTANCE_CM` | 20cm | Distance to detect walls (search mode) |
| `TARGET_DISTANCE_CM` | 10cm | Distance to stop at target |

**Examples:**
- Robot moves too fast? **Decrease** pulse durations
- Robot moves too slow? **Increase** pulse durations
- Robot crashes into walls? **Increase** OBSTACLE_DISTANCE_CM
- Robot stops too far from object? **Decrease** TARGET_DISTANCE_CM

### How to Change Parameters

1. Open the file in your code editor
2. Find the parameter (use Ctrl+F to search)
3. Change the value
4. Save the file
5. Upload to the ESP32

**Example:**
```cpp
// Before:
#define PULSE_DURATION 100

// After (slower turns):
#define PULSE_DURATION 75
```

---

## Part 3: Understanding Obstacle Avoidance

### How the ToF Sensor Works

The VL53L0X sensor shoots an invisible laser beam and measures how long it takes to bounce back.

```
        Laser
ToF ──────────────> Object
    <──────────────
        Reflection

Distance = Speed of Light × Time / 2
```

### Two Modes of Obstacle Detection

**1. Search Mode (Looking for Object)**
- Threshold: 20cm
- If wall detected: Turn away, continue searching
- Prevents crashing while spinning

**2. Tracking Mode (Moving Toward Object)**
- Threshold: 10cm
- If obstacle detected: Stop (assume we reached the target)
- Prevents crashing into the object

### The Code Logic

```
if (ToF distance < 10cm AND tracking target)
    → Stop! Target reached.

if (ToF distance < 20cm AND searching)
    → Turn away from wall, keep searching.
```

---

## Part 4: Manual Control via Web Interface

Your robot has a web interface for manual control:

### Open the Interface
1. Go to: `http://[ROVER-IP]/`
2. You'll see control buttons

### Available Controls

| Button | Action |
|--------|--------|
| FORWARD | Both motors forward (continuous) |
| BACKWARD | Both motors backward (continuous) |
| LEFT | Spin left |
| RIGHT | Spin right |
| PAUSE | Stop motors, ignore camera commands |
| RESUME | Resume autonomous mode + reset goal |
| SEARCH | Trigger search behavior |
| LIFT UP | Raise servo arm |
| LIFT DOWN | Lower servo arm |
| RESET GOAL | Reset the "goal reached" state on camera |

### PAUSE and RESUME

These are very useful for testing:
- **PAUSE**: Stops the robot and ignores commands from the camera
- **RESUME**: Starts listening to camera again AND resets the goal

**Tip:** Use PAUSE to stop the robot, then manually position it for testing.

---

## Part 5: The Navigation Logic

Here's how the autonomous system works:

```
┌─────────────────────────────────────────┐
│           ESP32-CAM Loop                │
├─────────────────────────────────────────┤
│  1. Capture camera frame                │
│  2. Run AI detection                    │
│  3. Did we detect the object?           │
│     │                                   │
│     ├── YES: Where is it in frame?      │
│     │   ├── Left side → TURN_LEFT       │
│     │   ├── Right side → TURN_RIGHT     │
│     │   └── Center → SLOW (forward)     │
│     │                                   │
│     └── NO (3x in a row):               │
│         └── SEARCH_LEFT or SEARCH_RIGHT │
│                                         │
│  4. Have we reached the goal?           │
│     (centered 3x AND slow 10x)          │
│     └── YES → GOAL (stop forever)       │
└─────────────────────────────────────────┘

┌─────────────────────────────────────────┐
│        ESP32 Dev Board Loop             │
├─────────────────────────────────────────┤
│  1. Receive command from camera         │
│  2. Check ToF sensor                    │
│     ├── Too close to wall? → Turn away  │
│     └── At target distance? → Stop      │
│  3. Execute motor command               │
│  4. Handle web interface requests       │
└─────────────────────────────────────────┘
```

---

## Part 6: Challenge Course!

Now it's time to test your robot!

### Challenge 1: Simple Follow

**Setup:**
1. Place your object on the floor
2. Position robot 1 meter away
3. Robot should find and approach the object

**Success criteria:** Robot stops within 15cm of object.

### Challenge 2: Search and Find

**Setup:**
1. Position robot facing away from object
2. Object should be behind and to the side

**Success criteria:** Robot searches, finds, and approaches object.

### Challenge 3: Obstacle Course

**Setup:**
1. Create a simple course with obstacles (boxes, books)
2. Place your object at the end
3. Robot must navigate around obstacles to reach object

**Success criteria:** Robot reaches object without crashing.

### Challenge 4: Moving Target

**Setup:**
1. Slowly move your object while robot follows
2. Try moving left and right

**Success criteria:** Robot tracks and follows the moving object.

### Bonus Challenge: Multiple Objects

If you have time:
1. Train your model on a second object
2. Test if the robot can distinguish between them

---

## Part 7: Going Further

### Ideas for Improvements

1. **Speed Control (PWM)**
   - Instead of just ON/OFF, control motor speed
   - Slow down when close to target

2. **Better Searching**
   - Remember which direction you last saw the object
   - Search in that direction first

3. **Multiple Objects**
   - Train on multiple objects
   - Different behaviors for different objects

4. **Sound Feedback**
   - Add a buzzer
   - Beep when object detected

5. **LED Indicators**
   - LED on when tracking
   - LED blinking when searching

### Learning Resources

- **Edge Impulse Docs:** https://docs.edgeimpulse.com
- **ESP32 Arduino Guide:** https://docs.espressif.com/projects/arduino-esp32
- **L298N Tutorial:** Search "L298N motor driver tutorial"

---

## Troubleshooting Reference

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Robot doesn't move | Wiring issue | Check motor connections |
| Robot spins forever | Can't detect object | Improve lighting, lower confidence |
| Robot crashes into walls | ToF not working | Check I2C wiring (SDA/SCL) |
| Robot overshoots | Moving too fast | Reduce pulse durations |
| Robot stops early | Goal threshold too low | Increase SLOW_THRESHOLD |
| Web interface not loading | WiFi issue | Check IP address, same network |
| False detections | Poor training data | Add more background images |
| Robot goes wrong way | Motor wires swapped | Swap wires at L298N |

---

## Congratulations!

You've built an autonomous robot that can:
- See using a camera
- Think using Machine Learning
- Navigate using obstacle avoidance
- Be controlled remotely via web interface

**Skills you've learned:**
- Basic electronics and wiring
- Microcontroller programming
- Machine Learning concepts
- Sensor integration
- Troubleshooting and debugging

**Keep experimenting!** Try new objects, new challenges, and new code modifications.

---

## Quick Reference Card

### Important IPs
- ESP32-CAM: `http://____________`
- ESP32 Dev Board: `http://____________`

### Key Files
- Camera code: `esp32cam/src/main.cpp`
- Motor code: `esp32_dev_board/src/main.cpp`

### Pin Reference
| Function | GPIO |
|----------|------|
| Left Forward | 13 |
| Left Backward | 14 |
| Right Forward | 33 |
| Right Backward | 32 |
| Servo | 25 |
| ToF SDA | 21 |
| ToF SCL | 22 |
| CAM TX → Rover RX | 14 → 16 |

### Web Commands
- PAUSE → Stop and ignore camera
- RESUME → Resume + reset goal
- RESET GOAL → Reset goal on camera only
