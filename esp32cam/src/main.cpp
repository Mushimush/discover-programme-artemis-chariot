/* Edge Impulse Object Detection on ESP32-CAM (AI-Thinker)
 * With Autonomous Centering, Motor Control, and WiFi Streaming
 *
 * Features:
 * - Edge Impulse object detection (on-device)
 * - Automatic centering and navigation
 * - Motor control for autonomous tracking
 * - WiFi MJPEG streaming for visualization
 *
 * ============================================================
 * PLATFORMIO SETUP:
 * Board: AI Thinker ESP32-CAM
 * ============================================================
 */

#include <Arduino.h>
#include <stanleytan-coaster-demo_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <WiFi.h>
#include <esp_http_server.h>

// AI-Thinker ESP32-CAM Pin Definition
#define CAMERA_MODEL_AI_THINKER

/* WiFi Configuration ------------------------------------------------------ */
// >>> CHANGE THESE TO YOUR WIFI CREDENTIALS <<<
const char* WIFI_SSID = "Munchi Family";
const char* WIFI_PASSWORD = "munchifatS1";

// const char* WIFI_SSID = "eee-iot";
// const char* WIFI_PASSWORD = "I0t@mar2026!";

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

/* Camera frame dimensions and centering constants -------------------------- */
#define FRAME_WIDTH                    320
#define FRAME_HEIGHT                   240
#define FRAME_CENTER_X                 160    // Center of 320px width
#define FRAME_CENTER_Y                 120    // Center of 240px height
#define CENTER_THRESHOLD_PX            20     // ±20px deadzone for "centered"

/* Distance estimation via bounding box size -------------------------------- */
#define STOP_SIZE_PERCENT              40     // Stop when bbox > 40% of frame (close)
#define SLOW_SIZE_PERCENT              25     // Slow down when bbox > 25% of frame

/* Detection confidence threshold ------------------------------------------- */
#define MIN_CONFIDENCE                 0.50   // Ignore detections below 50% confidence

/* Motor control pins (configure based on your motor driver) ---------------- */
// Note: GPIO 1, 3, 12, 13, 14, 15, 16 are available for motor control
// Avoid using camera pins!
#define MOTOR_LEFT_FORWARD_PIN         12
#define MOTOR_LEFT_BACKWARD_PIN        13
#define MOTOR_RIGHT_FORWARD_PIN        14
#define MOTOR_RIGHT_BACKWARD_PIN       15

// PWM Configuration for motor speed control
#define PWM_FREQ                       5000
#define PWM_RESOLUTION                 8      // 8-bit resolution (0-255)
#define PWM_CHANNEL_LF                 0
#define PWM_CHANNEL_LB                 1
#define PWM_CHANNEL_RF                 2
#define PWM_CHANNEL_RB                 3

// Motor speeds
#define MOTOR_SPEED_FORWARD            200    // 0-255 (adjust as needed)
#define MOTOR_SPEED_TURN               150    // Slower speed for turning
#define MOTOR_SPEED_SLOW               100    // Slow approach speed

/* Camera buffer constants -------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;

// Timing for detection
unsigned long last_inference_time = 0;
const unsigned long inference_interval = 500; // Run every 500ms for responsive control

// Goal reached state - stops autonomous navigation until reset
bool goalReached = false;

// Hybrid distance detection counters
int centeredCount = 0;   // Consecutive CENTER detections
int slowCount = 0;       // Total SLOW/approach commands sent while detecting
int noDetectionCount = 0; // Consecutive frames with no detection
String lastKnownDirection = "RIGHT"; // Last known direction of object (for smart search)
#define CENTERED_THRESHOLD 3   // Need 3 consecutive centered detections
#define SLOW_THRESHOLD 10      // Need 10 slow movements toward object
#define NO_DETECTION_THRESHOLD 3  // Need 3 consecutive misses before SEARCH

// WiFi streaming
httpd_handle_t stream_httpd = NULL;

// Latest detection results for JSON endpoint
struct DetectionResult {
    bool has_detection;
    char label[32];
    float confidence;
    uint32_t x, y, width, height;           // Original model-space coords
    int scaled_x, scaled_y, scaled_w, scaled_h;  // Frame-space coords
    int center_x, center_y;
    int offset_x, offset_y;
    int turn_percent;
    char direction[16];
    char command[16];
};
DetectionResult latest_detection = {};

// Viewer HTML served from ESP32
const char VIEWER_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32-CAM Detection Viewer</title>
<style>
body{background:#1a1a1a;margin:20px;font-family:sans-serif;color:#fff}
h3{margin:0 0 10px;color:#888;font-weight:normal}
#container{position:relative;display:inline-block}
#stream{display:block;background:#000}
#overlay{position:absolute;top:0;left:0;pointer-events:none}
#info{margin-top:10px;font-family:monospace;font-size:14px;color:#0f0}
</style>
</head>
<body>
<h3>ESP32-CAM Detection Viewer</h3>
<div id="container">
<img id="stream" src="/stream" width="320" height="240">
<canvas id="overlay" width="320" height="240"></canvas>
</div>
<div id="info">Waiting for detection...</div>
<script>
const canvas=document.getElementById('overlay');
const ctx=canvas.getContext('2d');
const info=document.getElementById('info');
let det=null,fc=0;

async function fetchDet(){
  try{
    const r=await fetch('/detection');
    det=await r.json();
    fc++;
  }catch(e){}
}

function draw(){
  const w=320,h=240,cx=160,cy=120;
  ctx.clearRect(0,0,w,h);
  ctx.strokeStyle='#0f0';ctx.lineWidth=2;
  ctx.beginPath();ctx.moveTo(cx-30,cy);ctx.lineTo(cx+30,cy);
  ctx.moveTo(cx,cy-30);ctx.lineTo(cx,cy+30);ctx.stroke();
  ctx.fillStyle='#0f0';ctx.beginPath();ctx.arc(cx,cy,5,0,Math.PI*2);ctx.fill();

  if(det&&det.detected){
    const b=det.bbox,c=det.center;
    ctx.strokeStyle='#f00';ctx.strokeRect(b.x,b.y,b.w,b.h);
    ctx.fillStyle='#000';ctx.beginPath();ctx.arc(c.x,c.y,8,0,Math.PI*2);ctx.fill();
    ctx.fillStyle='#ff0';ctx.beginPath();ctx.arc(c.x,c.y,6,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle='#f0f';ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(c.x,c.y);ctx.stroke();
    ctx.font='bold 14px monospace';ctx.fillStyle='#fff';
    ctx.fillText(det.label+' '+Math.round(det.confidence*100)+'%',10,20);
    ctx.font='12px monospace';ctx.fillStyle='#0ff';
    ctx.fillText('Offset:'+det.offset.x+'px Turn:'+det.turn_percent+'%',10,38);
    ctx.fillText(det.direction,10,54);
    info.innerHTML='<b>'+det.label+'</b> '+Math.round(det.confidence*100)+'% | Offset:'+det.offset.x+'px | Turn:'+det.turn_percent+'% | '+det.direction;
  }else{info.textContent='No detection';}
  ctx.font='10px monospace';ctx.fillStyle='#666';ctx.fillText('F:'+fc,280,15);
  requestAnimationFrame(draw);
}
setInterval(fetchDet,200);
draw();
</script>
</body>
</html>
)rawliteral";

/* Object position structure ----------------------------------------------- */
struct ObjectPosition {
    int center_x;           // Object center X coordinate
    int center_y;           // Object center Y coordinate
    int offset_x;           // Horizontal offset from frame center
    int offset_y;           // Vertical offset from frame center
    String direction;       // "LEFT", "CENTER", "RIGHT"
};

/* Camera configuration ---------------------------------------------------- */
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 10000000,  // Reduced from 20MHz to 10MHz for stability
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,    // 320x240

    .jpeg_quality = 12,
    .fb_count = 2,                        // Double buffering for streaming + detection
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,      // Always get latest frame for streaming
};

/* Function declarations --------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

// New functions for centering and motor control
void setupMotors(void);
ObjectPosition calculateObjectPosition(ei_impulse_result_bounding_box_t bb);
String generateNavigationCommand(ObjectPosition pos, int scaled_w, int scaled_h);
void executeMotorCommand(String command);
void stopAllMotors(void);

// WiFi and streaming functions
bool connectWiFi(void);
esp_err_t stream_handler(httpd_req_t *req);
esp_err_t detection_handler(httpd_req_t *req);
esp_err_t viewer_handler(httpd_req_t *req);
esp_err_t reset_handler(httpd_req_t *req);
bool startStreamServer(void);

/**
* @brief      Arduino setup function
*/
void setup()
{
    Serial.begin(115200);
    Serial2.begin(4800, SERIAL_8N1, -1, 14);  // RX=disabled, TX=GPIO14 (to ESP32 Rover)
    delay(3000);  // Longer delay to let power stabilize

    Serial.println("\n========================================");
    Serial.println("Starting up... please wait");
    Serial.println("========================================");
    Serial.println("ESP32-CAM Autonomous Object Following");
    Serial.println("Edge Impulse + Centering + Motor Control + WiFi Streaming");
    Serial.println("========================================\n");

    // Motor control disabled - ESP32 Rover handles motors via serial
    // setupMotors();
    Serial.println("Motor control disabled (ESP32 Rover handles motors)");

    // Initialize camera
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
    }

    // Connect to WiFi
    if (connectWiFi()) {
        // Start stream server
        if (startStreamServer()) {
            Serial.println("\n=== WiFi Streaming Ready ===");
            Serial.print("View stream at: http://");
            Serial.print(WiFi.localIP());
            Serial.println("/stream\n");
        } else {
            Serial.println("WARNING: Stream server failed (continuing with detection only)");
        }
    } else {
        Serial.println("WARNING: WiFi connection failed (continuing without streaming)");
    }

    ei_printf("\nStarting autonomous object detection and tracking...\n");
    delay(2000);
}

/**
* @brief      Get data and run inferencing with autonomous navigation
*/
void loop()
{
    // Run detection at specified interval
    unsigned long current_time = millis();
    if (current_time - last_inference_time < inference_interval) {
        delay(100);
        return;
    }

    last_inference_time = current_time;

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        stopAllMotors();
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        stopAllMotors();
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        stopAllMotors();
        return;
    }

    // print the predictions
    ei_printf("\n--- Inference Results ---\n");
    ei_printf("Timing: DSP %d ms, Classification %d ms, Anomaly %d ms\n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("\nObject detection bounding boxes:\n");
    bool found_objects = false;
    ObjectPosition best_object_pos;
    String navigation_command = "STOP";

    // If goal already reached, keep sending GOAL until reset
    if (goalReached) {
        ei_printf("  GOAL REACHED - waiting for reset\n");
        Serial.println("*** GOAL REACHED - WAITING FOR RESET ***");
        navigation_command = "GOAL";
        Serial2.println(navigation_command);
        free(snapshot_buf);
        return;
    }

    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }

        // Filter by confidence threshold
        if (bb.value < MIN_CONFIDENCE) {
            ei_printf("  Skipping low confidence detection: %s (%.0f%% < %.0f%%)\n",
                     bb.label, bb.value * 100, MIN_CONFIDENCE * 100);
            continue;
        }

        // Only respond to "matcha" - ignore other classes
        if (strcmp(bb.label, "matcha") != 0) {
            ei_printf("  Ignoring non-target: %s (only tracking matcha)\n", bb.label);
            continue;
        }

        found_objects = true;
        Serial.printf("*** OBJECT DETECTED: %s (%.0f%%) ***\n", bb.label, bb.value * 100);

        // Print detection info
        ei_printf("  Found: %s (%.2f confidence)\n", bb.label, bb.value);
        ei_printf("    Bounding Box: x=%u, y=%u, w=%u, h=%u\n",
                bb.x, bb.y, bb.width, bb.height);

        // Calculate object position and centering
        ObjectPosition pos = calculateObjectPosition(bb);

        ei_printf("    Object Center: (%d, %d)\n", pos.center_x, pos.center_y);
        ei_printf("    Frame Center: (%d, %d)\n", FRAME_CENTER_X, FRAME_CENTER_Y);
        ei_printf("    Offset from Frame Center: X=%d px, Y=%d px\n", pos.offset_x, pos.offset_y);

        // Calculate turn percentage (0-100% based on how far from center)
        int turn_percent = (abs(pos.offset_x) * 100) / FRAME_CENTER_X;
        if (turn_percent > 100) turn_percent = 100;
        ei_printf("    Turn Amount: %d%%\n", turn_percent);
        ei_printf("    Direction: %s\n", pos.direction.c_str());

        // For first/best detection, generate navigation command
        if (i == 0) {
            best_object_pos = pos;

            // Calculate scaled bounding box dimensions for distance estimation
            float scale_x = (float)FRAME_WIDTH / EI_CLASSIFIER_INPUT_WIDTH;
            float scale_y = (float)FRAME_HEIGHT / EI_CLASSIFIER_INPUT_HEIGHT;
            int scaled_w = (int)(bb.width * scale_x);
            int scaled_h = (int)(bb.height * scale_y);

            navigation_command = generateNavigationCommand(pos, scaled_w, scaled_h);
            ei_printf("    Navigation Command: %s\n", navigation_command.c_str());

            // Store detection data for JSON endpoint
            latest_detection.has_detection = true;
            strncpy(latest_detection.label, bb.label, sizeof(latest_detection.label) - 1);
            latest_detection.confidence = bb.value;
            latest_detection.x = bb.x;
            latest_detection.y = bb.y;
            latest_detection.width = bb.width;
            latest_detection.height = bb.height;
            latest_detection.scaled_x = (int)(bb.x * scale_x);
            latest_detection.scaled_y = (int)(bb.y * scale_y);
            latest_detection.scaled_w = scaled_w;
            latest_detection.scaled_h = scaled_h;
            latest_detection.center_x = pos.center_x;
            latest_detection.center_y = pos.center_y;
            latest_detection.offset_x = pos.offset_x;
            latest_detection.offset_y = pos.offset_y;
            latest_detection.turn_percent = turn_percent;
            strncpy(latest_detection.direction, pos.direction.c_str(), sizeof(latest_detection.direction) - 1);
            strncpy(latest_detection.command, navigation_command.c_str(), sizeof(latest_detection.command) - 1);
        }
    }

    if (!found_objects) {
        noDetectionCount++;
        ei_printf("  No objects detected (miss %d/%d)\n", noDetectionCount, NO_DETECTION_THRESHOLD);

        // Only send SEARCH after multiple consecutive misses
        if (noDetectionCount >= NO_DETECTION_THRESHOLD) {
            // Smart search: turn toward last known direction
            if (lastKnownDirection == "LEFT") {
                navigation_command = "SEARCH_LEFT";
            } else {
                navigation_command = "SEARCH_RIGHT";
            }
            ei_printf("  Navigation Command: %s (last seen %s)\n",
                     navigation_command.c_str(), lastKnownDirection.c_str());
            Serial.printf("  Smart search: turning %s\n", lastKnownDirection.c_str());

            // Reset hybrid distance counters when object truly lost
            centeredCount = 0;
            slowCount = 0;
            Serial.println("  Counters reset (object lost)");

            // Send SEARCH command
            Serial2.println(navigation_command);
            delay(400);  // Wait for turn to complete before next detection
        } else {
            // Keep moving forward slowly - might just be a brief miss
            navigation_command = "SLOW";
            ei_printf("  Navigation Command: %s (continuing forward)\n", navigation_command.c_str());
            Serial2.println(navigation_command);
        }

        // Clear detection data
        latest_detection.has_detection = false;
        strcpy(latest_detection.command, navigation_command.c_str());
    } else {
        // Object found - reset miss counter
        noDetectionCount = 0;

        // Send command with detection info: COMMAND|label|confidence
        String fullCommand = navigation_command + "|" +
                            String(latest_detection.label) + "|" +
                            String((int)(latest_detection.confidence * 100));
        Serial2.println(fullCommand);
    }

#else
    ei_printf("\nClassification results:\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: %.5f\n",
                ei_classifier_inferencing_categories[i],
                result.classification[i].value);
    }
    stopAllMotors(); // No object detection, stop motors
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("\nAnomaly score: %.3f\n", result.anomaly);
#endif

    ei_printf("-------------------------\n");

    free(snapshot_buf);

    // Keep WiFi alive (check occasionally)
    static unsigned long last_wifi_check = 0;
    if (millis() - last_wifi_check > 10000) {  // Check every 10 seconds
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected! Reconnecting...");
            connectWiFi();
        }
        last_wifi_check = millis();
    }
}

/**
 * @brief   Setup motor control pins and PWM
 */
void setupMotors(void) {
    // Configure PWM for motor control
    ledcSetup(PWM_CHANNEL_LF, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LB, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_RF, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_RB, PWM_FREQ, PWM_RESOLUTION);

    // Attach pins to PWM channels
    ledcAttachPin(MOTOR_LEFT_FORWARD_PIN, PWM_CHANNEL_LF);
    ledcAttachPin(MOTOR_LEFT_BACKWARD_PIN, PWM_CHANNEL_LB);
    ledcAttachPin(MOTOR_RIGHT_FORWARD_PIN, PWM_CHANNEL_RF);
    ledcAttachPin(MOTOR_RIGHT_BACKWARD_PIN, PWM_CHANNEL_RB);

    // Start with motors stopped
    stopAllMotors();
}

/**
 * @brief   Calculate object position and centering information
 *
 * @param   bb  Bounding box from Edge Impulse detection (in model input space)
 * @return  ObjectPosition structure with centering data (in frame space)
 */
ObjectPosition calculateObjectPosition(ei_impulse_result_bounding_box_t bb) {
    ObjectPosition pos;

    // Scale factors: model input (96x96) → camera frame (320x240)
    float scale_x = (float)FRAME_WIDTH / EI_CLASSIFIER_INPUT_WIDTH;   // 320/96 = 3.33
    float scale_y = (float)FRAME_HEIGHT / EI_CLASSIFIER_INPUT_HEIGHT; // 240/96 = 2.5

    // Scale bounding box from model space to frame space
    int scaled_x = (int)(bb.x * scale_x);
    int scaled_y = (int)(bb.y * scale_y);
    int scaled_w = (int)(bb.width * scale_x);
    int scaled_h = (int)(bb.height * scale_y);

    // Calculate object center in frame space
    pos.center_x = scaled_x + (scaled_w / 2);
    pos.center_y = scaled_y + (scaled_h / 2);

    // Calculate offset from frame center
    pos.offset_x = pos.center_x - FRAME_CENTER_X;
    pos.offset_y = pos.center_y - FRAME_CENTER_Y;

    // Determine direction based on horizontal offset
    if (abs(pos.offset_x) <= CENTER_THRESHOLD_PX) {
        pos.direction = "CENTER";
    } else if (pos.offset_x < 0) {
        pos.direction = "LEFT";
    } else {
        pos.direction = "RIGHT";
    }

    return pos;
}

/**
 * @brief   Generate navigation command based on object position and size
 *
 * @param   pos       ObjectPosition structure
 * @param   scaled_w  Bounding box width in frame space
 * @param   scaled_h  Bounding box height in frame space
 * @return  Navigation command string
 *
 * HYBRID DISTANCE DETECTION:
 * Since bounding box size doesn't reliably indicate distance, we use counters:
 * - centeredCount: Consecutive CENTER detections (object stable in view)
 * - slowCount: Total approach movements toward object
 * - GOAL triggers when: centeredCount >= 3 AND slowCount >= 10
 *
 * STEERING LOGIC:
 * - Object not centered → STEER_LEFT/RIGHT (one wheel only, gentle curve)
 * - Object centered → SLOW (both wheels, slow approach)
 */
String generateNavigationCommand(ObjectPosition pos, int scaled_w, int scaled_h) {
    // Calculate object size as percentage of frame (for logging only)
    int bbox_area = scaled_w * scaled_h;
    int frame_area = FRAME_WIDTH * FRAME_HEIGHT;  // 320 * 240 = 76800
    int size_percent = (bbox_area * 100) / frame_area;

    ei_printf("    Object Size: %d%% of frame\n", size_percent);
    Serial.printf("    Size: %d%% | Direction: %s\n", size_percent, pos.direction.c_str());

    // Update centeredCount: increment if centered, reset if not
    if (pos.direction == "CENTER") {
        centeredCount++;
    } else {
        centeredCount = 0;  // Reset if not centered
    }

    ei_printf("    Counters: centered=%d/%d, slow=%d/%d\n",
              centeredCount, CENTERED_THRESHOLD, slowCount, SLOW_THRESHOLD);
    Serial.printf("    Counters: centered=%d/%d, slow=%d/%d\n",
                  centeredCount, CENTERED_THRESHOLD, slowCount, SLOW_THRESHOLD);

    // HYBRID GOAL CHECK: Stable + approached enough times = GOAL!
    if (centeredCount >= CENTERED_THRESHOLD && slowCount >= SLOW_THRESHOLD) {
        Serial.println("*** GOAL REACHED - STABLE & APPROACHED! ***");
        ei_printf("    -> GOAL! Centered %d times, approached %d times\n",
                 centeredCount, slowCount);
        goalReached = true;
        centeredCount = 0;  // Reset counters
        slowCount = 0;
        return "GOAL";
    }

    // SIMPLE APPROACH: Object visible anywhere → go forward slowly
    // No steering - just move toward it. If we lose it, SEARCH will find it again.
    Serial.println("*** MATCHA VISIBLE - SLOW FORWARD ***");
    ei_printf("    -> Object visible, moving slowly forward\n");

    // Remember which side the object is on (for smart search if we lose it)
    if (pos.direction != "CENTER") {
        lastKnownDirection = pos.direction;
        Serial.printf("    Last seen: %s\n", lastKnownDirection.c_str());
    }

    slowCount++;  // Count as approach movement
    return "SLOW";
}

/**
 * @brief   Execute motor command
 *
 * @param   command  Navigation command string (FORWARD, TURN_LEFT, TURN_RIGHT, STOP)
 */
void executeMotorCommand(String command) {
    if (command == "FORWARD") {
        // Both motors forward at same speed
        ledcWrite(PWM_CHANNEL_LF, MOTOR_SPEED_FORWARD);
        ledcWrite(PWM_CHANNEL_LB, 0);
        ledcWrite(PWM_CHANNEL_RF, MOTOR_SPEED_FORWARD);
        ledcWrite(PWM_CHANNEL_RB, 0);

    } else if (command == "TURN_LEFT") {
        // Left motor slow/backward, right motor forward
        ledcWrite(PWM_CHANNEL_LF, 0);
        ledcWrite(PWM_CHANNEL_LB, MOTOR_SPEED_TURN);
        ledcWrite(PWM_CHANNEL_RF, MOTOR_SPEED_TURN);
        ledcWrite(PWM_CHANNEL_RB, 0);

    } else if (command == "TURN_RIGHT") {
        // Left motor forward, right motor slow/backward
        ledcWrite(PWM_CHANNEL_LF, MOTOR_SPEED_TURN);
        ledcWrite(PWM_CHANNEL_LB, 0);
        ledcWrite(PWM_CHANNEL_RF, 0);
        ledcWrite(PWM_CHANNEL_RB, MOTOR_SPEED_TURN);

    } else if (command == "SLOW") {
        // Both motors forward at slow speed
        ledcWrite(PWM_CHANNEL_LF, MOTOR_SPEED_SLOW);
        ledcWrite(PWM_CHANNEL_LB, 0);
        ledcWrite(PWM_CHANNEL_RF, MOTOR_SPEED_SLOW);
        ledcWrite(PWM_CHANNEL_RB, 0);

    } else if (command == "STOP") {
        stopAllMotors();
    }
}

/**
 * @brief   Stop all motors
 */
void stopAllMotors(void) {
    ledcWrite(PWM_CHANNEL_LF, 0);
    ledcWrite(PWM_CHANNEL_LB, 0);
    ledcWrite(PWM_CHANNEL_RF, 0);
    ledcWrite(PWM_CHANNEL_RB, 0);
}

/**
 * @brief   Connect to WiFi network
 *
 * @retval  true if connected successfully
 */
bool connectWiFi(void) {
    Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection failed!");
        return false;
    }

    Serial.println("WiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Stream URL: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/stream");

    return true;
}

/**
 * @brief   MJPEG stream handler
 *
 * @param   req  HTTP request
 * @retval  ESP_OK if successful
 */
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    Serial.println("Stream client connected");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;

        // Send MJPEG frame
        size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
        res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);

        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }

        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }

        esp_camera_fb_return(fb);

        if (res != ESP_OK) {
            break;
        }

        // Small delay to allow inference to grab frames
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    Serial.println("Stream client disconnected");
    return res;
}

/**
 * @brief   Detection data JSON endpoint
 *
 * @param   req  HTTP request
 * @retval  ESP_OK if successful
 */
esp_err_t detection_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char json[768];
    if (latest_detection.has_detection) {
        snprintf(json, sizeof(json),
            "{\"detected\":true,\"label\":\"%s\",\"confidence\":%.2f,"
            "\"bbox\":{\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d},"
            "\"frame\":{\"w\":%d,\"h\":%d,\"cx\":%d,\"cy\":%d},"
            "\"center\":{\"x\":%d,\"y\":%d},"
            "\"offset\":{\"x\":%d,\"y\":%d},"
            "\"turn_percent\":%d,"
            "\"direction\":\"%s\",\"command\":\"%s\"}",
            latest_detection.label, latest_detection.confidence,
            latest_detection.scaled_x, latest_detection.scaled_y,
            latest_detection.scaled_w, latest_detection.scaled_h,
            FRAME_WIDTH, FRAME_HEIGHT, FRAME_CENTER_X, FRAME_CENTER_Y,
            latest_detection.center_x, latest_detection.center_y,
            latest_detection.offset_x, latest_detection.offset_y,
            latest_detection.turn_percent,
            latest_detection.direction, latest_detection.command);
    } else {
        snprintf(json, sizeof(json),
            "{\"detected\":false,\"frame\":{\"w\":%d,\"h\":%d,\"cx\":%d,\"cy\":%d},\"command\":\"%s\"}",
            FRAME_WIDTH, FRAME_HEIGHT, FRAME_CENTER_X, FRAME_CENTER_Y,
            latest_detection.command);
    }

    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief   Serve viewer HTML page
 *
 * @param   req  HTTP request
 * @retval  ESP_OK if successful
 */
esp_err_t viewer_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, VIEWER_HTML, strlen(VIEWER_HTML));
}

/**
 * @brief   Reset goal state endpoint
 *
 * @param   req  HTTP request
 * @retval  ESP_OK if successful
 */
esp_err_t reset_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    goalReached = false;
    centeredCount = 0;
    slowCount = 0;
    Serial.println("*** GOAL RESET - RESUMING SEARCH ***");

    const char* response = "{\"status\":\"ok\",\"message\":\"Goal reset, resuming search\"}";
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief   Start HTTP stream server
 *
 * @retval  true if server started successfully
 */
bool startStreamServer(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t detection_uri = {
        .uri       = "/detection",
        .method    = HTTP_GET,
        .handler   = detection_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t viewer_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = viewer_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t reset_uri = {
        .uri       = "/reset",
        .method    = HTTP_GET,
        .handler   = reset_handler,
        .user_ctx  = NULL
    };

    Serial.println("Starting stream server on port 80");
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        httpd_register_uri_handler(stream_httpd, &detection_uri);
        httpd_register_uri_handler(stream_httpd, &viewer_uri);
        httpd_register_uri_handler(stream_httpd, &reset_uri);
        Serial.println("Stream server started successfully");
        Serial.println("Endpoints: / (viewer), /stream, /detection, /reset");
        return true;
    }

    Serial.println("Failed to start stream server");
    return false;
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
    }

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
