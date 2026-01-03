/*
 * ============================================================================
 * ESP32 ROVER MOTOR CONTROLLER
 * ============================================================================
 *
 * Receives navigation commands from ESP32-CAM via Serial
 * Controls L298N motor driver + servos + ToF sensor
 * Includes WiFi web interface for manual control
 *
 * PlatformIO Settings:
 *   Board: ESP32 Dev Module
 *   Framework: Arduino
 *
 * Required Libraries:
 *   - ESP32Servo
 *   - VL53L0X (Pololu)
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <VL53L0X.h>

// ============================================================================
// SECTION 1: CONFIGURATION
// ============================================================================

// ----- WiFi Credentials -----
// const char* WIFI_SSID = "eee-iot";
// const char* WIFI_PASSWORD = "I0t@mar2026!";

const char* WIFI_SSID = "Munchi Family";
const char* WIFI_PASSWORD = "munchifatS1";

// ----- Serial Communication -----
#define RXD2 16  // ESP32-CAM TX -> This pin
#define TXD2 17  // Unused

// ----- L298N Motor Pins -----
// IN1+IN2 (GPIO13+14) → OUT1/OUT2 → Left motors
// IN3+IN4 (GPIO33+32) → OUT3/OUT4 → Right motors
// NOTE: GPIO26 and GPIO27 are broken (stuck HIGH) on this board
#define LEFT_FWD   13  // IN1 - confirmed
#define LEFT_BWD   14  // IN2
#define RIGHT_FWD  33  // IN3 - confirmed
#define RIGHT_BWD  32  // IN4 - was GPIO27, now GPIO32

// ----- Servo Pins -----
#define LIFT_SERVO_PIN 25  // Lift arm servo (pan servo removed - GPIO13 now used for motor)

// ----- Servo Positions -----
#define SERVO_CENTER 70   // Calibrated center
#define SERVO_LEFT   130
#define SERVO_RIGHT  10
// NOTE: If lift servo moves opposite direction, swap LIFT_DOWN and LIFT_UP values
#define LIFT_DOWN    90
#define LIFT_UP      0
#define LIFT_MID     45

// ----- ToF Distance Thresholds (cm) -----
#define OBSTACLE_DISTANCE_CM 20  // Wall detection
#define TARGET_DISTANCE_CM   10  // Goal reached

// ----- Timing Constants -----
#define COMMAND_TIMEOUT      2000  // Stop if no command (ms)
#define TOF_LOG_INTERVAL     2000  // ToF logging interval (ms)
#define PULSE_DURATION       100   // Turn pulse (ms) - smaller = smoother search
#define STEER_PULSE_DURATION 75    // Steer pulse (ms) - was 150, reduced for LiPo
#define SLOW_PULSE_DURATION  125   // Slow forward pulse (ms) - was 250, reduced for LiPo

// ----- Logging -----
#define LOG_SIZE 10

// ============================================================================
// SECTION 2: GLOBAL VARIABLES
// ============================================================================

// ----- Hardware Objects -----
WebServer server(80);
Servo liftServoMotor;
VL53L0X tofSensor;

// ----- State Variables -----
bool tofAvailable = false;
bool isTrackingTarget = false;
bool isSearching = false;
bool isPaused = false;  // When true, ignore ESP32-CAM commands
int searchDirection = 1;
int currentServoAngle = SERVO_CENTER;
String currentStatus = "STOPPED";
String esp32CamIP = "";

// ----- Timing -----
unsigned long lastCommandTime = 0;
unsigned long lastTofLog = 0;

// ----- Command Logging -----
String commandLog[LOG_SIZE];
int logIndex = 0;
String tofLog[LOG_SIZE];
int tofLogIndex = 0;

// ============================================================================
// SECTION 3: FUNCTION DECLARATIONS
// ============================================================================

// Setup
void setupWiFi();
void setupWebServer();

// Web handlers
void handleRoot();
void handleCommand();
void handleStatus();
void handleLog();
void handleTof();

// Commands
void processCommand(String cmd);
void addToLog(String source, String cmd);
void addToTofLog(int distance);

// Motors
void forward();
void backward();
void turnLeft();
void turnRight();
void steerLeft();
void steerRight();
void stopMotors();

// Servos
void liftArm(int angle);

// Sensors
bool isObstacleAhead();

// ============================================================================
// SECTION 4: SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    Serial2.begin(4800, SERIAL_8N1, RXD2, TXD2);

    // Motor pins
    pinMode(LEFT_FWD, OUTPUT);
    pinMode(LEFT_BWD, OUTPUT);
    pinMode(RIGHT_FWD, OUTPUT);
    pinMode(RIGHT_BWD, OUTPUT);
    stopMotors();

    // Lift servo
    ESP32PWM::allocateTimer(0);
    liftServoMotor.setPeriodHertz(50);
    liftServoMotor.attach(LIFT_SERVO_PIN, 500, 2400);
    liftServoMotor.write(LIFT_DOWN);

    // ToF sensor
    Wire.begin();
    if (tofSensor.init()) {
        tofAvailable = true;
        tofSensor.setTimeout(200);
        Serial.println("ToF sensor OK");
    } else {
        Serial.println("ToF sensor NOT detected");
    }

    Serial.println("\n================================");
    Serial.println("ESP32 Rover Motor Controller");
    Serial.println("================================\n");

    setupWiFi();
    setupWebServer();

    Serial.println("\nWiring:");
    Serial.println("  L298N: IN1=13, IN2=14, IN3=26, IN4=27");
    Serial.println("  Servo: Lift=25");
    Serial.println("  Serial: RX=16 (from ESP32-CAM)");
    Serial.println("  ToF: SDA=21, SCL=22\n");
}

void setupWiFi() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected! IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi failed - continuing without");
    }
}

void setupWebServer() {
    server.on("/", handleRoot);
    server.on("/cmd", handleCommand);
    server.on("/status", handleStatus);
    server.on("/log", handleLog);
    server.on("/tof", handleTof);
    server.begin();
    Serial.println("Web server started");
}

// ============================================================================
// SECTION 5: MAIN LOOP
// ============================================================================

void loop() {
    server.handleClient();

    // Commands from ESP32-CAM (ignored when paused)
    if (Serial2.available()) {
        String rawCmd = Serial2.readStringUntil('\n');
        rawCmd.trim();
        if (rawCmd.length() > 0 && !isPaused) {
            String cmd = rawCmd;
            String logMsg = rawCmd;
            bool hasDetection = false;

            // Parse: COMMAND|label|confidence
            int sep1 = rawCmd.indexOf('|');
            if (sep1 > 0) {
                cmd = rawCmd.substring(0, sep1);
                int sep2 = rawCmd.indexOf('|', sep1 + 1);
                if (sep2 > 0) {
                    String label = rawCmd.substring(sep1 + 1, sep2);
                    String conf = rawCmd.substring(sep2 + 1);
                    logMsg = label + " (" + conf + "%) -> " + cmd;
                    hasDetection = true;
                }
            }

            isTrackingTarget = hasDetection;
            Serial.print("CAM: ");
            Serial.println(logMsg);
            addToLog("CAM", logMsg);
            processCommand(cmd);
        }
    }

    // Commands from USB (testing)
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) {
            Serial.print("USB: ");
            processCommand(cmd);
        }
    }

    // Safety timeout
    if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
        stopMotors();
        isSearching = false;
    }

    // ToF logging
    if (millis() - lastTofLog > TOF_LOG_INTERVAL) {
        lastTofLog = millis();
        if (tofAvailable) {
            int distance = tofSensor.readRangeSingleMillimeters();
            if (!tofSensor.timeoutOccurred()) {
                int distCm = distance / 10;
                addToTofLog(distCm);
                Serial.print("[ToF] ");
                Serial.print(distCm);
                Serial.println(" cm");
            }
        }
    }
}

// ============================================================================
// SECTION 6: WEB SERVER HANDLERS
// ============================================================================

void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Rover Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; text-align: center; margin: 20px; background: #1a1a2e; color: white; }
        h1 { color: #00d4ff; }
        .btn { padding: 20px 40px; margin: 10px; font-size: 18px; border: none; border-radius: 10px; cursor: pointer; min-width: 120px; }
        .btn-move { background: #4CAF50; color: white; }
        .btn-turn { background: #2196F3; color: white; }
        .btn-stop { background: #f44336; color: white; }
        .btn-resume { background: #8bc34a; color: white; }
        .btn-other { background: #ff9800; color: white; }
        .btn-lift { background: #00bcd4; color: white; }
        .btn-reset { background: #9c27b0; color: white; }
        .btn:active { transform: scale(0.95); }
        .row { margin: 5px; }
        #status { padding: 15px; margin: 20px; background: #16213e; border-radius: 5px; font-size: 20px; }
        #status.goal { background: #1b5e20; color: #4caf50; font-size: 28px; font-weight: bold; }
        input[type=range] { width: 200px; }
        #log { background:#0d1117; padding:10px; border-radius:5px; font-family:monospace; font-size:12px; text-align:left; max-width:300px; margin:10px auto; height:80px; overflow-y:auto; }
    </style>
</head>
<body>
    <h1>Rover Control</h1>
    <div id="status">Loading...</div>
    <div class="row"><button class="btn btn-move" onclick="send('FORWARD')">FORWARD</button></div>
    <div class="row">
        <button class="btn btn-turn" onclick="send('TURN_LEFT')">LEFT</button>
        <button class="btn btn-stop" onclick="send('STOP')">PAUSE</button>
        <button class="btn btn-resume" onclick="resume()">RESUME</button>
        <button class="btn btn-turn" onclick="send('TURN_RIGHT')">RIGHT</button>
    </div>
    <div class="row"><button class="btn btn-move" onclick="send('BACKWARD')">BACKWARD</button></div>
    <div class="row">
        <button class="btn btn-other" onclick="send('SEARCH')">SEARCH</button>
        <button class="btn btn-lift" onclick="send('LIFT_UP')">LIFT UP</button>
        <button class="btn btn-lift" onclick="send('LIFT_DOWN')">LIFT DOWN</button>
    </div>
    <div class="row" style="margin-top:15px;">
        <input type="text" id="camIP" placeholder="CAM IP (e.g. 192.168.1.100)" style="padding:10px;width:180px;border-radius:5px;border:none;">
        <button class="btn btn-reset" onclick="resetGoal()">RESET GOAL</button>
    </div>
    <div id="log"></div>
    <script>
        function send(a){fetch('/cmd?action='+a).then(r=>r.text()).then(t=>document.getElementById('status').innerText=t);}
        function resetGoal(){
            var ip=document.getElementById('camIP').value.trim();
            if(!ip){alert('Enter CAM IP first');return;}
            localStorage.setItem('camIP',ip);
            fetch('http://'+ip+'/reset',{mode:'no-cors'});
        }
        function resume(){
            send('RESUME');
            var ip=localStorage.getItem('camIP');
            if(ip){fetch('http://'+ip+'/reset',{mode:'no-cors'});}
        }
        document.getElementById('camIP').value=localStorage.getItem('camIP')||'';
        setInterval(()=>{
            fetch('/status').then(r=>r.text()).then(t=>document.getElementById('status').innerText=t);
            fetch('/log').then(r=>r.text()).then(t=>document.getElementById('log').innerHTML=t.split('\n').map(l=>l?'<div>'+l+'</div>':'').join(''));
        },500);
    </script>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}

void handleCommand() {
    String action = server.arg("action");
    if (action.length() > 0) {
        processCommand(action);
        server.send(200, "text/plain", currentStatus);
    } else {
        server.send(400, "text/plain", "Missing action");
    }
}

void handleStatus() {
    server.send(200, "text/plain", currentStatus);
}

void handleLog() {
    String log = "";
    for (int i = 0; i < LOG_SIZE; i++) {
        int idx = (logIndex - 1 - i + LOG_SIZE) % LOG_SIZE;
        if (commandLog[idx].length() > 0) {
            log += commandLog[idx] + "\n";
        }
    }
    server.send(200, "text/plain", log);
}

void handleTof() {
    String log = "";
    for (int i = 0; i < LOG_SIZE; i++) {
        int idx = (tofLogIndex - 1 - i + LOG_SIZE) % LOG_SIZE;
        if (tofLog[idx].length() > 0) {
            log += tofLog[idx] + "\n";
        }
    }
    server.send(200, "text/plain", log);
}

// ============================================================================
// SECTION 7: COMMAND PROCESSING
// ============================================================================

void addToLog(String source, String cmd) {
    commandLog[logIndex] = source + ": " + cmd;
    logIndex = (logIndex + 1) % LOG_SIZE;
}

void addToTofLog(int distance) {
    tofLog[tofLogIndex] = String(distance) + " cm";
    tofLogIndex = (tofLogIndex + 1) % LOG_SIZE;
}

void processCommand(String cmd) {
    lastCommandTime = millis();
    isSearching = false;
    Serial.println(cmd);

    if (cmd == "FORWARD") {
        forward();
        currentStatus = "FORWARD";
    }
    else if (cmd == "BACKWARD") {
        backward();
        currentStatus = "BACKWARD";
    }
    else if (cmd == "TURN_LEFT") {
        turnLeft();
        delay(PULSE_DURATION);
        stopMotors();
        currentStatus = "TURN_LEFT";
    }
    else if (cmd == "TURN_RIGHT") {
        turnRight();
        delay(PULSE_DURATION);
        stopMotors();
        currentStatus = "TURN_RIGHT";
    }
    else if (cmd == "STOP") {
        stopMotors();
        isPaused = true;
        currentStatus = "PAUSED";
    }
    else if (cmd == "RESUME") {
        isPaused = false;
        currentStatus = "RESUMED";
    }
    else if (cmd == "SLOW") {
        forward();
        delay(SLOW_PULSE_DURATION);
        stopMotors();
        currentStatus = "SLOW";
    }
    else if (cmd == "STEER_LEFT") {
        steerLeft();
        delay(STEER_PULSE_DURATION);
        stopMotors();
        currentStatus = "STEER_LEFT";
    }
    else if (cmd == "STEER_RIGHT") {
        steerRight();
        delay(STEER_PULSE_DURATION);
        stopMotors();
        currentStatus = "STEER_RIGHT";
    }
    else if (cmd == "GOAL") {
        stopMotors();
        currentStatus = "GOAL REACHED!";
        Serial.println("*** GOAL REACHED! ***");
    }
    else if (cmd == "SEARCH" || cmd == "SPIN" || cmd == "SEARCH_RIGHT") {
        if (tofAvailable) {
            int dist = tofSensor.readRangeSingleMillimeters() / 10;
            if (dist > 0 && dist < OBSTACLE_DISTANCE_CM) {
                turnLeft();
                delay(PULSE_DURATION * 2);
                stopMotors();
                currentStatus = "AVOIDING_WALL";
                return;
            }
        }
        turnRight();
        delay(PULSE_DURATION);
        stopMotors();
        currentStatus = "SEARCH_RIGHT";
    }
    else if (cmd == "SEARCH_LEFT") {
        if (tofAvailable) {
            int dist = tofSensor.readRangeSingleMillimeters() / 10;
            if (dist > 0 && dist < OBSTACLE_DISTANCE_CM) {
                turnRight();
                delay(PULSE_DURATION * 2);
                stopMotors();
                currentStatus = "AVOIDING_WALL";
                return;
            }
        }
        turnLeft();
        delay(PULSE_DURATION);
        stopMotors();
        currentStatus = "SEARCH_LEFT";
    }
    else if (cmd == "LIFT_UP") {
        liftArm(LIFT_UP);
        currentStatus = "LIFT_UP";
    }
    else if (cmd == "LIFT_DOWN") {
        liftArm(LIFT_DOWN);
        currentStatus = "LIFT_DOWN";
    }
    else if (cmd.startsWith("LIFT ")) {
        int angle = cmd.substring(5).toInt();
        liftArm(angle);
        currentStatus = "LIFT " + String(angle);
    }
    else if (cmd == "IP" || cmd == "STATUS") {
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    }
    else if (cmd == "TOF") {
        if (tofAvailable) {
            int dist = tofSensor.readRangeSingleMillimeters() / 10;
            Serial.print("ToF: ");
            Serial.print(dist);
            Serial.println(" cm");
        }
    }
    else if (cmd == "LEFT_ONLY") {
        digitalWrite(LEFT_FWD, HIGH);
        digitalWrite(LEFT_BWD, LOW);
        digitalWrite(RIGHT_FWD, LOW);
        digitalWrite(RIGHT_BWD, LOW);
    }
    else if (cmd == "RIGHT_ONLY") {
        digitalWrite(LEFT_FWD, LOW);
        digitalWrite(LEFT_BWD, LOW);
        digitalWrite(RIGHT_FWD, HIGH);
        digitalWrite(RIGHT_BWD, LOW);
    }
    else if (cmd == "TEST") {
        Serial.println("\n=== MOTOR TEST ===");

        Serial.println("1. LEFT FORWARD (GPIO26)...");
        digitalWrite(LEFT_FWD, HIGH);
        digitalWrite(LEFT_BWD, LOW);
        digitalWrite(RIGHT_FWD, LOW);
        digitalWrite(RIGHT_BWD, LOW);
        delay(2000);
        stopMotors();
        delay(500);

        Serial.println("2. LEFT BACKWARD (GPIO27)...");
        digitalWrite(LEFT_FWD, LOW);
        digitalWrite(LEFT_BWD, HIGH);
        digitalWrite(RIGHT_FWD, LOW);
        digitalWrite(RIGHT_BWD, LOW);
        delay(2000);
        stopMotors();
        delay(500);

        Serial.println("3. RIGHT FORWARD (GPIO13)...");
        digitalWrite(LEFT_FWD, LOW);
        digitalWrite(LEFT_BWD, LOW);
        digitalWrite(RIGHT_FWD, HIGH);
        digitalWrite(RIGHT_BWD, LOW);
        delay(2000);
        stopMotors();
        delay(500);

        Serial.println("4. RIGHT BACKWARD (GPIO14)...");
        digitalWrite(LEFT_FWD, LOW);
        digitalWrite(LEFT_BWD, LOW);
        digitalWrite(RIGHT_FWD, LOW);
        digitalWrite(RIGHT_BWD, HIGH);
        delay(2000);
        stopMotors();

        Serial.println("=== TEST COMPLETE ===\n");
        currentStatus = "TEST_DONE";
    }
    else {
        currentStatus = "UNKNOWN: " + cmd;
    }
}

// ============================================================================
// SECTION 8: MOTOR CONTROL
// ============================================================================

void forward() {
    if (tofAvailable) {
        int dist = tofSensor.readRangeSingleMillimeters() / 10;
        if (isTrackingTarget && dist > 0 && dist < TARGET_DISTANCE_CM) {
            Serial.println("TARGET REACHED!");
            stopMotors();
            currentStatus = "TARGET_REACHED";
            return;
        }
        if (!isTrackingTarget && dist > 0 && dist < OBSTACLE_DISTANCE_CM) {
            Serial.println("WALL BLOCKED!");
            stopMotors();
            currentStatus = "WALL_BLOCKED";
            return;
        }
    }
    digitalWrite(LEFT_FWD, HIGH);
    digitalWrite(LEFT_BWD, LOW);
    digitalWrite(RIGHT_FWD, HIGH);
    digitalWrite(RIGHT_BWD, LOW);
}

void backward() {
    digitalWrite(LEFT_FWD, LOW);
    digitalWrite(LEFT_BWD, HIGH);
    digitalWrite(RIGHT_FWD, LOW);
    digitalWrite(RIGHT_BWD, HIGH);
}

void turnLeft() {
    digitalWrite(LEFT_FWD, LOW);
    digitalWrite(LEFT_BWD, HIGH);
    digitalWrite(RIGHT_FWD, HIGH);
    digitalWrite(RIGHT_BWD, LOW);
}

void turnRight() {
    digitalWrite(LEFT_FWD, HIGH);
    digitalWrite(LEFT_BWD, LOW);
    digitalWrite(RIGHT_FWD, LOW);
    digitalWrite(RIGHT_BWD, HIGH);
}

void steerLeft() {
    digitalWrite(LEFT_FWD, LOW);
    digitalWrite(LEFT_BWD, LOW);
    digitalWrite(RIGHT_FWD, HIGH);
    digitalWrite(RIGHT_BWD, LOW);
}

void steerRight() {
    digitalWrite(LEFT_FWD, HIGH);
    digitalWrite(LEFT_BWD, LOW);
    digitalWrite(RIGHT_FWD, LOW);
    digitalWrite(RIGHT_BWD, LOW);
}

void stopMotors() {
    digitalWrite(LEFT_FWD, LOW);
    digitalWrite(LEFT_BWD, LOW);
    digitalWrite(RIGHT_FWD, LOW);
    digitalWrite(RIGHT_BWD, LOW);
}

// ============================================================================
// SECTION 9: SERVO CONTROL
// ============================================================================

void liftArm(int angle) {
    angle = constrain(angle, 0, 180);
    liftServoMotor.write(angle);
}


// ============================================================================
// SECTION 10: SENSOR FUNCTIONS
// ============================================================================

bool isObstacleAhead() {
    if (!tofAvailable) return false;

    int distance = tofSensor.readRangeSingleMillimeters();
    if (tofSensor.timeoutOccurred()) return false;

    int distCm = distance / 10;
    return (distCm > 0 && distCm < OBSTACLE_DISTANCE_CM);
}
