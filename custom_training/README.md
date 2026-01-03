# Custom Object Detection Training Guide

Train Edge Impulse to detect your own custom objects for the rover!

## Step 1: Capture Training Images (30-60 minutes)

Take 100-300 photos of EACH object you want to detect.

### Option A: Web Browser (Recommended)

**Requirements:** Chrome or Edge browser

1. **Find ESP32-CAM IP:** Open Arduino Serial Monitor, reset ESP32-CAM, look for IP address
2. **Open capture tool:** Double-click `capture.html` to open in browser
3. **Connect:** Enter `http://[IP]/stream` and click **Connect**
4. **Select folder:** Click **Select Folder** to choose where images save
5. **Capture:** Press **S** or click button - images save immediately to folder
6. **Vary your shots:** Move object, change angle/lighting, capture again
7. **New class:** Change class name, keep capturing (creates new subfolder)
8. **Goal:** 100-200 images per object

Images are saved as: `[folder]/[class]/[class]_0001_timestamp.jpg`

### Option B: Python Script (Advanced)

Requires Python 3 with OpenCV.

```bash
# Create virtual environment (optional but recommended)
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run capture tool
python capture_training_images.py --stream-url http://192.168.1.100/stream
```

**Controls:**
- `S` - Save image
- `N` - Start new class
- `Q` - Quit

---

## Step 2: Upload to Edge Impulse

Edge Impulse handles labeling and training - no local GPU needed!

1. Go to https://studio.edgeimpulse.com (free account)
2. Create new project > Select **Images** > **Object Detection**
3. Go to **Data acquisition** > Upload your images
4. Click each image > Draw bounding boxes around objects
5. Assign class labels (e.g., "matcha")

---

## Step 3: Create and Train Model

1. Go to **Create impulse**
2. Add **Image** block (96x96 recommended for ESP32-CAM)
3. Add **Object Detection** block
4. Click **Generate features**
5. Click **Start training** (takes 5-20 minutes)

---

## Step 4: Deploy to ESP32-CAM

1. Go to **Deployment**
2. Select **Arduino library**
3. Click **Build**
4. Download the `.zip` file
5. Extract to `esp32cam/lib/` folder
6. Update the library name in `esp32cam/src/main.cpp` if needed
7. Upload to ESP32-CAM

---

## Tips for Good Training Data

- **Variety is key!**
  - Different angles (top, side, diagonal)
  - Different lighting (bright, dim, shadows)
  - Various backgrounds
  - Different distances from camera
  - Multiple orientations

- **Minimum dataset sizes:**
  - Minimum: 50 images per class
  - Recommended: 150-200 images per class
  - Best: 300+ images per class

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Low accuracy | Collect more images, add variety |
| False detections | Add "background" images with no objects |
| Objects not detected | Check bounding boxes are accurate |
| Model too slow | Use smaller image size (96x96) |

---

## Need Help?

- Edge Impulse Docs: https://docs.edgeimpulse.com
- ESP32-CAM Guide: https://docs.edgeimpulse.com/docs/edge-ai-hardware/mcu/espressif-esp32
