# Day 2: Teaching Your Robot to See

Today you'll teach your robot to recognize objects using Machine Learning!

---

## What is Machine Learning?

Imagine teaching a friend to recognize your pet dog:
1. You show them many photos of your dog
2. You say "this is my dog" each time
3. After enough examples, they can recognize your dog anywhere

**Machine Learning works the same way:**
1. We show the computer many images of an object
2. We label each image ("this is a ball", "this is a cup")
3. The computer learns patterns and can recognize the object in new images

The more examples you give, the better it learns!

---

## Today's Plan

1. **Upload code** to both ESP32 boards
2. **Capture images** of your object using the camera
3. **Upload to Edge Impulse** and label them
4. **Train the AI model**
5. **Deploy** the model to your robot
6. **Test** your robot!

---

## Part 1: Upload the Code

### What You'll Need
- USB cable (for programming)
- Computer with **VS Code** and **PlatformIO extension** installed

### Step 1.0: Install VS Code + PlatformIO (if not done)

**If already installed, skip to Step 1.1**

1. Download VS Code: https://code.visualstudio.com
2. Install and open VS Code
3. Click the **Extensions** icon in the left sidebar (looks like 4 squares)
4. Search for "PlatformIO"
5. Click **Install** on "PlatformIO IDE"
6. Wait for installation (may take a few minutes)
7. Restart VS Code when prompted

### Step 1.1: Download the Project Code

1. Go to: **https://github.com/YOUR-USERNAME/RoverProject**
2. Click the green **Code** button
3. Click **Download ZIP**
4. Extract the ZIP file to your Desktop or Documents folder

### Step 1.2: Open the Rover Project

1. Open VS Code
2. Click **File** → **Open Folder...**
3. Navigate to the extracted folder, then open: `esp32_dev_board`
4. Click **Open** (or **Select Folder** on Mac)
5. Wait for PlatformIO to load (you'll see activity in the bottom bar)

When ready, you'll see a toolbar at the bottom of VS Code:
```
┌─────────────────────────────────────────────────────┐
│  🏠  ✓  →  🔌  🗑️    esp32dev                      │
│      │  │   │                                       │
│      │  │   └── Serial Monitor (plug icon)          │
│      │  └────── Upload (arrow icon) ← USE THIS      │
│      └────────── Build (checkmark)                  │
└─────────────────────────────────────────────────────┘
```

### Step 1.3: Update WiFi Credentials

Before uploading, set your WiFi network name and password.

1. In VS Code, open the file: `src/main.cpp`
2. Find these lines near the top (around line 36):
```cpp
const char* WIFI_SSID = "YourWiFiName";
const char* WIFI_PASSWORD = "YourWiFiPassword";
```
3. Change them to your WiFi network name and password
4. Save the file (**Ctrl+S** or **Cmd+S**)

### Step 1.4: Upload to ESP32 Dev Board

1. Connect ESP32 Dev Board to your computer via USB
2. In VS Code, click the **→ (arrow)** button in the bottom toolbar
3. Wait for the upload to complete
   - You'll see progress in the Terminal panel
   - Look for "SUCCESS" at the end
4. Click the **🔌 (plug)** icon to open Serial Monitor
5. You should see the IP address printed

**Write down the IP address:** `http://___.___.___.___ `

### Step 1.5: Upload to ESP32-CAM

1. **Close** the current folder: **File** → **Close Folder**
2. Open the ESP32-CAM folder: **File** → **Open Folder...** → select `esp32cam`
3. Wait for PlatformIO to load
4. Open `src/main.cpp` and update WiFi credentials (same as before)
5. Save the file
6. Disconnect ESP32 Dev Board, connect ESP32-CAM via USB
7. Click the **→ (arrow)** button to upload
8. Click the **🔌 (plug)** icon for Serial Monitor
9. Note the IP address

**Write down the IP address:** `http://___.___.___.___ `

**Troubleshooting Upload:**
- If upload fails, try pressing the **RESET** button on ESP32-CAM during "Connecting..."
- Some ESP32-CAM boards need GPIO0 connected to GND during upload

---

## Part 2: Test the Connection

### Test ESP32 Dev Board Web Interface

1. Open a web browser
2. Go to `http://[ESP32-DEV-BOARD-IP]/`
3. You should see a control panel with buttons
4. Click **FORWARD** - do the motors spin?
5. Click **STOP**

### Test ESP32-CAM Stream

1. In your browser, go to `http://[ESP32-CAM-IP]/stream`
2. You should see a live video feed from the camera!

**Troubleshooting:**
- If no connection: Check WiFi credentials
- If motors don't work: Check wiring from Day 1

---

## Part 3: Capture Training Images

You'll use a special tool to capture images of your object.

### Step 3.1: Open the Capture Tool

1. Find the file: `custom_training/capture.html`
2. Double-click to open in Chrome or Edge browser

### Step 3.2: Connect to Camera

1. Enter the stream URL: `http://[ESP32-CAM-IP]/stream`
2. Click **Connect**
3. You should see the live camera feed

### Step 3.3: Choose Your Object

Pick an object you want your robot to follow:
- A tennis ball
- A red cup
- A toy
- Anything with a distinct color/shape!

### Step 3.4: Select Save Folder

1. Click **Select Folder**
2. Create a new folder called "training_images"
3. Select it

### Step 3.5: Set Class Name

1. In the "Class Name" field, type a name for your object
   - Example: `ball`, `red_cup`, `toy_car`
   - Use lowercase, no spaces (use underscore `_` instead)

### Step 3.6: Capture Images!

Now capture **100-200 images** of your object:

1. Hold your object in front of the camera
2. Press **S** key or click **Capture**
3. Move the object slightly
4. Capture again
5. Repeat!

**Tips for good training data:**
- [ ] Different angles (front, side, top, tilted)
- [ ] Different distances (close, medium, far)
- [ ] Different backgrounds (table, floor, hand)
- [ ] Different lighting (bright, dim, shadows)
- [ ] Object in different positions in frame (center, left, right)

**Goal:** 150-200 images minimum

### Step 3.7: Capture "Background" Images

Also capture ~50 images with NO object visible:
1. Change class name to `background`
2. Capture images of empty backgrounds
3. This helps the AI know when the object is NOT present

---

## Part 4: Train on Edge Impulse

Edge Impulse is a free online platform for training AI models.

### Step 4.1: Create Account

1. Go to: **https://studio.edgeimpulse.com**
2. Click **Sign Up** (it's free!)
3. Create an account

### Step 4.2: Create New Project

1. Click **Create new project**
2. Name it something like "Robot Object Detection"
3. Select **Images** as the data type
4. Select **Object Detection** (not classification!)

### Step 4.3: Upload Images

1. Click **Data acquisition** in the left menu
2. Click **Upload data**
3. Select your training images folder
4. Upload all images

### Step 4.4: Label Your Images

This is the most important step! You need to draw boxes around your objects.

1. Click on an image
2. Draw a box around your object
3. Enter the label (e.g., "ball")
4. Click **Save**
5. Repeat for ALL images

**Tips:**
- Draw the box tight around the object
- Be consistent with your labels
- Don't label background images (leave them empty)

### Step 4.5: Create Impulse

1. Click **Create impulse** in the left menu
2. Add an **Image** block
   - Set to 96x96 pixels (good for ESP32-CAM)
3. Add **Object Detection** block
4. Click **Save Impulse**

### Step 4.6: Generate Features

1. Click **Image** in the left menu
2. Click **Generate features**
3. Wait for it to complete

### Step 4.7: Train the Model

1. Click **Object detection** in the left menu
2. Set training cycles to 100 (or more for better accuracy)
3. Click **Start training**
4. Wait 5-20 minutes

### Step 4.8: Test Your Model

1. Click **Model testing** in the left menu
2. Click **Classify all**
3. Check the accuracy - aim for 80%+

---

## Part 5: Deploy to Your Robot

### Step 5.1: Download Arduino Library

1. Click **Deployment** in the left menu
2. Select **Arduino library**
3. Click **Build**
4. Download the `.zip` file

### Step 5.2: Install the Library

1. Extract the `.zip` file
2. Copy the folder to: `esp32cam/lib/`
3. The folder name will be something like `your-project-name_inferencing`

### Step 5.3: Update Code (if needed)

If your library name is different from the template:

**File:** `esp32cam/src/main.cpp`

Find this line:
```cpp
#include <your-project-name_inferencing.h>
```

Update it to match your library folder name.

### Step 5.4: Upload to ESP32-CAM

1. Connect ESP32-CAM to computer
2. Upload the updated code
3. Open Serial Monitor

You should see detection messages when you point the camera at your object!

---

## Part 6: First Test Run!

### Test 1: Detection Only

1. Point the ESP32-CAM at your object
2. Watch the Serial Monitor
3. You should see: `Detected: [your_object] (85%)`

### Test 2: Manual Control

1. Open the rover web interface: `http://[ROVER-IP]/`
2. Test the buttons: FORWARD, BACKWARD, LEFT, RIGHT
3. Make sure motors respond correctly

### Test 3: Autonomous Mode

1. Place your object on the floor
2. Position the rover a few feet away
3. The rover should:
   - Search by spinning when it doesn't see the object
   - Move toward the object when it detects it
   - Stop when it reaches the object

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Low detection accuracy | Capture more images, vary angles/lighting |
| False detections | Add more "background" images |
| Robot doesn't move | Check wiring, check motor test on web interface |
| Robot moves wrong direction | Swap motor wires at L298N |
| "No objects detected" | Lower confidence threshold in code |

---

## Understanding the Code

### ESP32-CAM (main.cpp)

The camera code does this in a loop:
1. Capture a frame from camera
2. Run the AI model on the frame
3. If object detected:
   - Calculate where it is (left, center, right)
   - Send command to rover (SLOW, TURN_LEFT, etc.)
4. If no object detected for 3 frames:
   - Send SEARCH command

### ESP32 Dev Board (main.cpp)

The rover code does this in a loop:
1. Listen for commands from ESP32-CAM
2. Execute the command (move motors)
3. Check ToF sensor for obstacles
4. Provide web interface for manual control

---

## What's Next?

Tomorrow (Day 3), you'll:
1. Fine-tune parameters for better performance
2. Learn about obstacle avoidance
3. Create a challenge course for your robot!

---

## Glossary

| Term | Meaning |
|------|---------|
| Machine Learning | Teaching computers to learn from examples |
| Object Detection | Finding and locating objects in images |
| Inference | Running a trained model on new data |
| Confidence | How sure the AI is (0-100%) |
| Epoch | One complete pass through all training data |
| Dataset | Collection of images used for training |
