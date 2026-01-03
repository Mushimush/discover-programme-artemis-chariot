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
 * ARDUINO IDE SETUP:
 * 1. Board: AI Thinker ESP32-CAM
 * 2. Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)
 * 3. Install your Edge Impulse library via:
 *    Sketch -> Include Library -> Add .ZIP Library
 * ============================================================
 */

#include <stanleytan-coaster-demo_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <WiFi.h>
#include <esp_http_server.h>

// AI-Thinker ESP32-CAM Pin Definition
#define CAMERA_MODEL_AI_THINKER

/* WiFi Configuration ------------------------------------------------------ */
// >>> CHANGE THESE TO YOUR WIFI CREDENTIALS <<<
// const char* WIFI_SSID = "Munchi Family";
// const char* WIFI_PASSWORD = "munchifatS1";

const char* WIFI_SSID = "eee-iot";
const char* WIFI_PASSWORD = "I0t@mar2026!";

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
#define FRAME_CENTER_X                 160
#define FRAME_CENTER_Y                 120
#define CENTER_THRESHOLD_PX            20

/* Distance estimation via bounding box size -------------------------------- */
#define STOP_SIZE_PERCENT              40
#define SLOW_SIZE_PERCENT              25

/* Detection confidence threshold ------------------------------------------- */
#define MIN_CONFIDENCE                 0.50

/* Motor control pins (disabled - ESP32 Rover handles motors) --------------- */
#define MOTOR_LEFT_FORWARD_PIN         12
#define MOTOR_LEFT_BACKWARD_PIN        13
#define MOTOR_RIGHT_FORWARD_PIN        14
#define MOTOR_RIGHT_BACKWARD_PIN       15

#define PWM_FREQ                       5000
#define PWM_RESOLUTION                 8
#define PWM_CHANNEL_LF                 0
#define PWM_CHANNEL_LB                 1
#define PWM_CHANNEL_RF                 2
#define PWM_CHANNEL_RB                 3

#define MOTOR_SPEED_FORWARD            200
#define MOTOR_SPEED_TURN               150
#define MOTOR_SPEED_SLOW               100

/* Camera buffer constants -------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;

unsigned long last_inference_time = 0;
const unsigned long inference_interval = 500;

bool goalReached = false;

int centeredCount = 0;
int slowCount = 0;
int noDetectionCount = 0;
String lastKnownDirection = "RIGHT";
#define CENTERED_THRESHOLD 3
#define SLOW_THRESHOLD 10
#define NO_DETECTION_THRESHOLD 3

httpd_handle_t stream_httpd = NULL;

struct DetectionResult {
    bool has_detection;
    char label[32];
    float confidence;
    uint32_t x, y, width, height;
    int scaled_x, scaled_y, scaled_w, scaled_h;
    int center_x, center_y;
    int offset_x, offset_y;
    int turn_percent;
    char direction[16];
    char command[16];
};
DetectionResult latest_detection = {};

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

struct ObjectPosition {
    int center_x;
    int center_y;
    int offset_x;
    int offset_y;
    String direction;
};

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
    .xclk_freq_hz = 10000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,
};

// Function declarations
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void setupMotors(void);
ObjectPosition calculateObjectPosition(ei_impulse_result_bounding_box_t bb);
String generateNavigationCommand(ObjectPosition pos, int scaled_w, int scaled_h);
void executeMotorCommand(String command);
void stopAllMotors(void);
bool connectWiFi(void);
esp_err_t stream_handler(httpd_req_t *req);
esp_err_t detection_handler(httpd_req_t *req);
esp_err_t viewer_handler(httpd_req_t *req);
esp_err_t reset_handler(httpd_req_t *req);
bool startStreamServer(void);

void setup() {
    Serial.begin(115200);
    Serial2.begin(4800, SERIAL_8N1, -1, 14);  // TX=GPIO14 (to ESP32 Rover)
    delay(3000);

    Serial.println("\n========================================");
    Serial.println("ESP32-CAM Object Detection");
    Serial.println("========================================\n");

    Serial.println("Motor control disabled (ESP32 Rover handles motors)");

    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    } else {
        ei_printf("Camera initialized\r\n");
    }

    if (connectWiFi()) {
        if (startStreamServer()) {
            Serial.println("\n=== WiFi Streaming Ready ===");
            Serial.print("View stream at: http://");
            Serial.print(WiFi.localIP());
            Serial.println("/stream\n");
        }
    }

    ei_printf("\nStarting object detection...\n");
    delay(2000);
}

void loop() {
    unsigned long current_time = millis();
    if (current_time - last_inference_time < inference_interval) {
        delay(100);
        return;
    }
    last_inference_time = current_time;

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
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

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        stopAllMotors();
        return;
    }

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool found_objects = false;
    ObjectPosition best_object_pos;
    String navigation_command = "STOP";

    if (goalReached) {
        navigation_command = "GOAL";
        Serial2.println(navigation_command);
        free(snapshot_buf);
        return;
    }

    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0 || bb.value < MIN_CONFIDENCE) continue;
        if (strcmp(bb.label, "matcha") != 0) continue;

        found_objects = true;
        ObjectPosition pos = calculateObjectPosition(bb);

        if (i == 0) {
            best_object_pos = pos;
            float scale_x = (float)FRAME_WIDTH / EI_CLASSIFIER_INPUT_WIDTH;
            float scale_y = (float)FRAME_HEIGHT / EI_CLASSIFIER_INPUT_HEIGHT;
            int scaled_w = (int)(bb.width * scale_x);
            int scaled_h = (int)(bb.height * scale_y);
            navigation_command = generateNavigationCommand(pos, scaled_w, scaled_h);

            latest_detection.has_detection = true;
            strncpy(latest_detection.label, bb.label, sizeof(latest_detection.label) - 1);
            latest_detection.confidence = bb.value;
            latest_detection.scaled_x = (int)(bb.x * scale_x);
            latest_detection.scaled_y = (int)(bb.y * scale_y);
            latest_detection.scaled_w = scaled_w;
            latest_detection.scaled_h = scaled_h;
            latest_detection.center_x = pos.center_x;
            latest_detection.center_y = pos.center_y;
            latest_detection.offset_x = pos.offset_x;
            latest_detection.offset_y = pos.offset_y;
            int turn_percent = (abs(pos.offset_x) * 100) / FRAME_CENTER_X;
            latest_detection.turn_percent = turn_percent > 100 ? 100 : turn_percent;
            strncpy(latest_detection.direction, pos.direction.c_str(), sizeof(latest_detection.direction) - 1);
            strncpy(latest_detection.command, navigation_command.c_str(), sizeof(latest_detection.command) - 1);
        }
    }

    if (!found_objects) {
        noDetectionCount++;
        if (noDetectionCount >= NO_DETECTION_THRESHOLD) {
            navigation_command = (lastKnownDirection == "LEFT") ? "SEARCH_LEFT" : "SEARCH_RIGHT";
            centeredCount = 0;
            slowCount = 0;
            Serial2.println(navigation_command);
            delay(400);
        } else {
            navigation_command = "SLOW";
            Serial2.println(navigation_command);
        }
        latest_detection.has_detection = false;
        strcpy(latest_detection.command, navigation_command.c_str());
    } else {
        noDetectionCount = 0;
        String fullCommand = navigation_command + "|" + String(latest_detection.label) + "|" + String((int)(latest_detection.confidence * 100));
        Serial2.println(fullCommand);
    }
#endif

    free(snapshot_buf);

    static unsigned long last_wifi_check = 0;
    if (millis() - last_wifi_check > 10000) {
        if (WiFi.status() != WL_CONNECTED) connectWiFi();
        last_wifi_check = millis();
    }
}

void setupMotors(void) {
    ledcSetup(PWM_CHANNEL_LF, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LB, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_RF, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_RB, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_LEFT_FORWARD_PIN, PWM_CHANNEL_LF);
    ledcAttachPin(MOTOR_LEFT_BACKWARD_PIN, PWM_CHANNEL_LB);
    ledcAttachPin(MOTOR_RIGHT_FORWARD_PIN, PWM_CHANNEL_RF);
    ledcAttachPin(MOTOR_RIGHT_BACKWARD_PIN, PWM_CHANNEL_RB);
    stopAllMotors();
}

ObjectPosition calculateObjectPosition(ei_impulse_result_bounding_box_t bb) {
    ObjectPosition pos;
    float scale_x = (float)FRAME_WIDTH / EI_CLASSIFIER_INPUT_WIDTH;
    float scale_y = (float)FRAME_HEIGHT / EI_CLASSIFIER_INPUT_HEIGHT;
    int scaled_x = (int)(bb.x * scale_x);
    int scaled_y = (int)(bb.y * scale_y);
    int scaled_w = (int)(bb.width * scale_x);
    int scaled_h = (int)(bb.height * scale_y);
    pos.center_x = scaled_x + (scaled_w / 2);
    pos.center_y = scaled_y + (scaled_h / 2);
    pos.offset_x = pos.center_x - FRAME_CENTER_X;
    pos.offset_y = pos.center_y - FRAME_CENTER_Y;
    if (abs(pos.offset_x) <= CENTER_THRESHOLD_PX) pos.direction = "CENTER";
    else if (pos.offset_x < 0) pos.direction = "LEFT";
    else pos.direction = "RIGHT";
    return pos;
}

String generateNavigationCommand(ObjectPosition pos, int scaled_w, int scaled_h) {
    if (pos.direction == "CENTER") centeredCount++;
    else centeredCount = 0;

    if (centeredCount >= CENTERED_THRESHOLD && slowCount >= SLOW_THRESHOLD) {
        goalReached = true;
        centeredCount = 0;
        slowCount = 0;
        return "GOAL";
    }

    if (pos.direction != "CENTER") lastKnownDirection = pos.direction;
    slowCount++;
    return "SLOW";
}

void executeMotorCommand(String command) {
    // Disabled - ESP32 Rover handles motors
}

void stopAllMotors(void) {
    ledcWrite(PWM_CHANNEL_LF, 0);
    ledcWrite(PWM_CHANNEL_LB, 0);
    ledcWrite(PWM_CHANNEL_RF, 0);
    ledcWrite(PWM_CHANNEL_RB, 0);
}

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
    if (WiFi.status() != WL_CONNECTED) return false;
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    return true;
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    char * part_buf[64];
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) { res = ESP_FAIL; break; }
        size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, fb->len);
        res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    return res;
}

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
            "\"turn_percent\":%d,\"direction\":\"%s\",\"command\":\"%s\"}",
            latest_detection.label, latest_detection.confidence,
            latest_detection.scaled_x, latest_detection.scaled_y,
            latest_detection.scaled_w, latest_detection.scaled_h,
            FRAME_WIDTH, FRAME_HEIGHT, FRAME_CENTER_X, FRAME_CENTER_Y,
            latest_detection.center_x, latest_detection.center_y,
            latest_detection.offset_x, latest_detection.offset_y,
            latest_detection.turn_percent, latest_detection.direction, latest_detection.command);
    } else {
        snprintf(json, sizeof(json),
            "{\"detected\":false,\"frame\":{\"w\":%d,\"h\":%d,\"cx\":%d,\"cy\":%d},\"command\":\"%s\"}",
            FRAME_WIDTH, FRAME_HEIGHT, FRAME_CENTER_X, FRAME_CENTER_Y, latest_detection.command);
    }
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

esp_err_t viewer_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, VIEWER_HTML, strlen(VIEWER_HTML));
}

esp_err_t reset_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    goalReached = false;
    centeredCount = 0;
    slowCount = 0;
    return httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
}

bool startStreamServer(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;
    httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler };
    httpd_uri_t detection_uri = { .uri = "/detection", .method = HTTP_GET, .handler = detection_handler };
    httpd_uri_t viewer_uri = { .uri = "/", .method = HTTP_GET, .handler = viewer_handler };
    httpd_uri_t reset_uri = { .uri = "/reset", .method = HTTP_GET, .handler = reset_handler };
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        httpd_register_uri_handler(stream_httpd, &detection_uri);
        httpd_register_uri_handler(stream_httpd, &viewer_uri);
        httpd_register_uri_handler(stream_httpd, &reset_uri);
        return true;
    }
    return false;
}

bool ei_camera_init(void) {
    if (is_initialised) return true;
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) return false;
    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }
    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_camera_deinit();
    is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) return false;
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
    if (!converted) return false;
    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        ei::image::processing::crop_and_interpolate_rgb888(out_buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS, out_buf, img_width, img_height);
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;
    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
