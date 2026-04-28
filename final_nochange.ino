/*
 * ESP32C3 GP02, MPU6050, UART, and BLE Monitor - RTOS Version
 * @author : Ziad-Elmekawy (Consolidated)
 * @date   : 29, jan, 2026
 * * NOTE: All sensor/data acquisition functions (Tasks 1-5) are independent 
 * * and run continuously, regardless of the BLE connection status.
 */

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <math.h>
#include "MAX30105.h"
#include "heartRate.h"

#include "spo2_algorithm.h"

// --- BLE Libraries ---
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
// --- Display Libraries ---
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <time.h>

// ------------ BLE Configuration ------------
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
const size_t JSON_DOC_SIZE = 512;

// ------------ Hardware Configurations ------------
// GPS Module (HardwareSerial 1)
#define GPS_RX 20
#define GPS_TX 21
// --- TFT PINS ---
#define TFT_CS 10
#define TFT_DC 1
#define TFT_RST 5
// --- Battery pin ---
#define Battery_Pin 2
// --- COLORS ---
#define COLOR_GOLD 0xFD20
#define COLOR_RED 0xF800
#define COLOR_CYAN 0x04FF
#define COLOR_WHITE 0xFFFF
#define COLOR_BLACK 0x0000
#define COLOR_GREY 0x528A
#define COLOR_LIGHTGREY 0xBDF7
// Fall Detection Constants
const unsigned long FALL_DETECTION_TIMEOUT_MS = 20000;

// ------------ Objects ------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
HardwareSerial otherSerial(2);
MPU6050 mpu;
MAX30105 particleSensor;
Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_RST);

// ------------ FreeRTOS Handles & Events ------------
SemaphoreHandle_t xDataMutex;
SemaphoreHandle_t xSerialMutex;
EventGroupHandle_t bluetoothEvents;
SemaphoreHandle_t xI2CMutex;
#define BLUETOOTH_SEND_NOW_BIT (1 << 0)

// ------------ Globals (Shared Data) ------------
// Local Data (Collected & Sent via BLE)
volatile int globalStepCount = 0;

// Remote Data (Received via UART & Sent via BLE)
volatile int globalHR = 0;
volatile int globalSpO2 = 0;
volatile int globalTemp = 37;
volatile bool globalSOS = false;

// Motion & Fall Detection Flags (Sent via BLE)
volatile bool globalImpact = false;  // True if a Shake/Impact just happened
volatile bool globalStill = false;   // True if device has been still for > 3s

// MPU and Motion Variables
volatile unsigned long lastImpactTime = 0;
float prevAx = 0, prevAy = 0, prevAz = 0;
unsigned long lastMotionTime = 0;
unsigned long lastStepTime = 0;
unsigned char screen_status = 'n';
// Step Counter
int stepGoal = 10000;
int displayedSteps = -1;
float lastMag = 0;
bool stepState = false;
// Heart rate and oxygen
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;
enum PostureType {
  POSTURE_UNKNOWN,
  POSTURE_SITTING,
  POSTURE_STANDING
};

volatile PostureType globalPosture = POSTURE_UNKNOWN;
#define STANDING_Z_MIN 0.85
#define SITTING_Z_MAX 0.65
#define POSTURE_STABLE_MS 3000

enum ActivityType {
  ACTIVITY_IDLE,
  ACTIVITY_WALKING,
  ACTIVITY_RUNNING
};

volatile ActivityType globalActivity = ACTIVITY_IDLE;

// heart rate
#define HR_MIN_VALID 40
#define HR_MAX_VALID 180
#define HR_AVG_WINDOW 5
int hrHistory[HR_AVG_WINDOW];
uint8_t hrIndex = 0;
bool hrBufferFull = false;
int validReadingsCount = 0;
int beatsDetectedTotal = 0;
int lastValidHR = 0;
int beatAvg = 0;
byte rateSpot = 0;
int finalBPM = 0;
long lastBeat = 0;
float beatsPerMinute;
const byte RATE_SIZE = 8;
byte rates[RATE_SIZE];
// SpO2 Vars
bool o2Locked = false;
int finalO2 = 0;
double avgRed = 0, avgIR = 0, sumRedRMS = 0, sumIRRMS = 0;
// Blood oxygen
bool bpmLocked = false;
#define SPO2_MIN_VALID 90
#define SPO2_MAX_VALID 100
#define SPO2_AVG_WINDOW 6

int spo2History[SPO2_AVG_WINDOW];
uint8_t spo2Index = 0;
bool spo2BufferFull = false;
int lastValidSpO2 = 0;

// Sensor State
enum SensorState { STATE_WAITING_FINGER,
                   STATE_MEASURING,
                   STATE_LOCKED };
SensorState currentState = STATE_WAITING_FINGER;
SensorState lastState = STATE_LOCKED;

/************ Buttons ************/
// Back Button
#define BACK_BUTTON_PIN 3
#define FORWARD_BUTTON_PIN 0
#define BACK_BUTTON_HOLD_TIME_MS 3000
volatile bool backButtonPressed = false;
typedef enum {
  SCREEN_CLOCK = 0,
  SCREEN_HEART,
  SCREEN_SPO2,
  SCREEN_STEPS,
  SCREEN_SPEED,
  SCREEN_COUNT  // = 5
} ScreenType;

volatile ScreenType currentScreen = SCREEN_CLOCK;
volatile bool screenChanged = true;

// ========== MEDICINE ALERT ==========
volatile bool medAlertActive = false;
unsigned long medAlertStartTime = 0;

#define MED_ALERT_DURATION_MS 2000  // Alert time


/************ Vibration motor ************/
#define VIBRATION_PIN 7
// Constants
const float motionThreshold = 0.15;
const float shakeThreshold = 0.5;
const unsigned long stillDelay = 3000;
/************ GPS GP02 module ************/
struct tm gpsTime;
bool gpsTimeValid = false;
volatile double globalLatitude = 31.043621;  // Mansoura (default)
volatile double globalLongitude = 31.358102;
volatile float globalSpeed = 0.0;
unsigned long lastGPSUpdate = 0;
const int TIMEZONE_OFFSET = 2;  // مصر UTC +2
TaskHandle_t gpsTaskHandle = NULL;

/************ Display ************/
int lastMinute = -1;
unsigned long lastInteractionTime = 0;

/************ SOS Variables ************/
bool sosActive = false;
unsigned long lastSosBlink = 0;
bool sosRedState = true;
/************ BLE Variables ************/
volatile bool globalFindMyWatch = false;

// ------------ BLE Callback Classes ------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    xSemaphoreTake(xSerialMutex, portMAX_DELAY);
    Serial.println("Device connected. [Sensor tasks continue to run]");
    xSemaphoreGive(xSerialMutex);
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    pServer->getAdvertising()->start();
    xSemaphoreTake(xSerialMutex, portMAX_DELAY);
    Serial.println("Device disconnected. Advertising restarted. [Sensor tasks continue to run]");
    xSemaphoreGive(xSerialMutex);
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();
    rxValue.trim();
    rxValue.toUpperCase();

    if (rxValue.length() > 0) {

      xSemaphoreTake(xSerialMutex, portMAX_DELAY);
      Serial.print("[BLE RX] Received: ");
      Serial.println(rxValue);
      xSemaphoreGive(xSerialMutex);

      // -------- FIND MY WATCH COMMANDS --------
      if (rxValue == "FIND_ON") {
        globalFindMyWatch = true;
        xEventGroupSetBits(bluetoothEvents, BLUETOOTH_SEND_NOW_BIT);
      }

      else if (rxValue == "FIND_OFF") {
        globalFindMyWatch = false;
      }

      else if (rxValue == "MED") {

        medAlertActive = true;
        medAlertStartTime = millis();

        // vibrate for 1 second
        digitalWrite(VIBRATION_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1000));
        digitalWrite(VIBRATION_PIN, LOW);
      }
    }
  }
};
/* ============================================================================================== */
/* ------------------------------------ Function Prototypes ------------------------------------- */
/* ============================================================================================== */
// All sensor/data acquisition functions are implemented as FreeRTOS Tasks
void TaskGPS(void *pvParameters);
void TaskMPU(void *pvParameters);
void TaskUARTControl(void *pvParameters);
void TaskUARTReceive(void *pvParameters);
void TaskBLESend(void *pvParameters);
void TaskMAX30102(void *pvParameters);
void TaskBackButton(void *pvParameters);
void TaskFindMyWatch(void *pvParameters);
// Helper function
int getSmoothedHR(int newValue);
int getSmoothedSpO2(int newValue);
void vibrateMotor(int duration_ms);
void drawSOSScreen();
void drawClockScreen();
void drawFindMyWatchScreen();

/* ============================================================================================== */
/* ------------------------------------ MAIN SETUP AND LOOP ------------------------------------- */
/* ============================================================================================== */
void setup() {
  Serial.begin(115200);

  pinMode(Battery_Pin , INPUT_PULLUP);

  // 1. Initialize Hardware (MPU, GPS, BackButton , ForwardButton, Vibration motor)
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Wire.begin(8, 9);
  mpu.initialize();
  pinMode(BACK_BUTTON_PIN, INPUT_PULLUP);
  pinMode(FORWARD_BUTTON_PIN, INPUT_PULLUP);
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);  // OFF initially
  // TFT init
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(COLOR_BLACK);
  lastInteractionTime = millis();


  // 2. Initialize BLE
  BLEDevice::init("Health Monitor");
  BLEDevice::setMTU(200);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  // 3. Create FreeRTOS Handles
  xDataMutex = xSemaphoreCreateMutex();
  xSerialMutex = xSemaphoreCreateMutex();
  bluetoothEvents = xEventGroupCreate();
  xI2CMutex = xSemaphoreCreateMutex();

  // 4. Create Tasks (Sensor tasks run independently)
  xTaskCreate(TaskGPS, "GPS_Task", 4096, NULL, 1, &gpsTaskHandle);
  xTaskCreate(TaskMPU, "MPU_Task", 4096, NULL, 2, NULL);
  xTaskCreate(TaskMAX30102, "MAX30102_Task", 8192, NULL, 1, NULL);
  xTaskCreate(TaskBackButton, "Back_Button", 2048, NULL, 2, NULL);  // Higher Priority for emergency
  xTaskCreate(TaskForwardButton, "Forward_Button", 2048, NULL, 1, NULL);
  xTaskCreatePinnedToCore(TaskDisplay, "Display Task", 4096, NULL, 1, NULL, 0);

  xTaskCreate(TaskBLESend, "BLE_Send", 8192, NULL, 2, NULL);
  xTaskCreate(TaskFindMyWatch, "FindMyWatch", 2048, NULL, 3, NULL);
}

void loop() {
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = true;
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}

/* ================================================================ */
/* ----------------------- RTOS TASKS ----------------------------- */
/* ================================================================ */
// --- TASK 1: GPS Parsing (INDEPENDENT) ---
void TaskGPS(void *pvParameters) {
  for (;;) {

    while (gpsSerial.available() > 0) {

      if (gps.encode(gpsSerial.read())) {

        // ---------- LOCATION ----------
        if (gps.location.isValid() && gps.location.isUpdated()) {

          if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {

            globalLatitude = gps.location.lat();
            globalLongitude = gps.location.lng();

            xSemaphoreGive(xDataMutex);
          }
        }

        // ---------- SPEED ----------
        if (gps.speed.isValid()) {

          if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {

            globalSpeed = gps.speed.kmph();

            xSemaphoreGive(xDataMutex);
          }
        } else {
          globalSpeed = 0.0;
        }

        // ---------- DATE & TIME ----------
        if (gps.date.isValid() && gps.time.isValid()) {

          if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {

            gpsTime.tm_year = gps.date.year() - 1900;
            gpsTime.tm_mon = gps.date.month() - 1;
            gpsTime.tm_mday = gps.date.day();

            gpsTime.tm_hour = gps.time.hour() + 2;  // Egypt UTC +2
            gpsTime.tm_min = gps.time.minute();
            gpsTime.tm_sec = gps.time.second();

            if (gpsTime.tm_hour >= 24)
              gpsTime.tm_hour -= 24;

            gpsTimeValid = true;

            xSemaphoreGive(xDataMutex);
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// --- TASK 2: MPU Logic & Fall Detection (INDEPENDENT) ---
void TaskMPU(void *pvParameters) {
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  static bool fallWarningPrinted = false;
  static unsigned long lastSensorPrintTime = 0;
  const unsigned long sensorPrintInterval = 5000;

  for (;;) {
    unsigned long now = millis();
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    float accX = ax_raw / 16384.0;
    float accY = ay_raw / 16384.0;
    float accZ = az_raw / 16384.0;
    float dX = abs(accX - prevAx);
    float dY = abs(accY - prevAy);
    float dZ = abs(accZ - prevAz);
    float delta = max(dX, max(dY, dZ));
    static unsigned long postureStableStart = 0;
    static PostureType postureCandidate = POSTURE_UNKNOWN;

    // Only detect posture when user is still
    if (delta < motionThreshold) {

      if (accZ > STANDING_Z_MIN) {
        postureCandidate = POSTURE_STANDING;
      } else if (accZ < SITTING_Z_MAX) {
        postureCandidate = POSTURE_SITTING;
      } else {
        postureCandidate = POSTURE_UNKNOWN;
      }

      if (postureStableStart == 0) {
        postureStableStart = millis();
      }

      if (millis() - postureStableStart >= POSTURE_STABLE_MS) {
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
          globalPosture = postureCandidate;
          xSemaphoreGive(xDataMutex);
        }
      }

    } else {
      postureStableStart = 0;  // user moved
    }

    // Step Count Logic
    float diff = accZ - prevAz;
    if (diff > 0.25 && (now - lastStepTime) > 300) {
      lastStepTime = now;
      if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
        globalStepCount++;
        xSemaphoreGive(xDataMutex);
      }
    }
    // Walking or running
    static unsigned long stepWindowStart = 0;
    static int stepWindowCount = 0;

    if (diff > 0.25 && (now - lastStepTime) > 300) {

      lastStepTime = now;
      stepWindowCount++;

      if (stepWindowStart == 0) {
        stepWindowStart = now;
      }

      if (now - stepWindowStart >= 5000) {  // 5 sec window

        float stepsPerMin = (stepWindowCount * 60000.0) / (now - stepWindowStart);

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {

          if (stepsPerMin > 140) {
            globalActivity = ACTIVITY_RUNNING;
          } else if (stepsPerMin > 60) {
            globalActivity = ACTIVITY_WALKING;
          } else {
            globalActivity = ACTIVITY_IDLE;
          }

          xSemaphoreGive(xDataMutex);
        }

        stepWindowCount = 0;
        stepWindowStart = 0;
      }
    }

    // Motion/Still Logic and IMPACT DETECTION (Shake)
    bool currentImpact = false;
    bool currentStill = false;

    if (delta >= shakeThreshold) {
      lastMotionTime = now;
      lastImpactTime = now;
      fallWarningPrinted = false;
      currentImpact = true;
    } else if (delta > motionThreshold) {
      lastMotionTime = now;
      fallWarningPrinted = false;
    } else if (now - lastMotionTime > stillDelay) {
      currentStill = true;

      // FALL DETECTION WARNING LOGIC (20 seconds)
      if (lastImpactTime > 0 && (now - lastImpactTime >= FALL_DETECTION_TIMEOUT_MS) && !fallWarningPrinted) {
        xSemaphoreTake(xSerialMutex, portMAX_DELAY);
        Serial.println("!!! URGENT WARNING: Possible Fall - Device Still for 20s !!!");
        xSemaphoreGive(xSerialMutex);
        fallWarningPrinted = true;
        xEventGroupSetBits(bluetoothEvents, BLUETOOTH_SEND_NOW_BIT);
      }
    }
    // Update global status flags
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
      globalImpact = currentImpact;  // This is the 'Shake' flag (IMP)
      globalStill = currentStill;
      xSemaphoreGive(xDataMutex);
    }

    // Debug printout to show sensor activity independent of BLE
    if (now - lastSensorPrintTime >= sensorPrintInterval) {
      lastSensorPrintTime = now;
      xSemaphoreTake(xSerialMutex, portMAX_DELAY);
      xSemaphoreGive(xSerialMutex);
    }

    prevAx = accX;
    prevAy = accY;
    prevAz = accZ;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


// --- TASK 4: BLE Send (SENDS ALL REQUIRED DATA) ---
void TaskBLESend(void *pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(5000);
  DynamicJsonDocument doc(JSON_DOC_SIZE);
  uint8_t count  = 0 ;
  // Structure to hold a safe copy of all data points for transmission
  struct {
    int hr, o2, steps;
    double lat, lon;
    float bat, speed;
    bool sos, impact, still;
  } dataToSend;

  for (;;) {
    // Wait for 5 seconds OR an emergency event (non-blocking to sensor tasks)
    xEventGroupWaitBits(
      bluetoothEvents,
      BLUETOOTH_SEND_NOW_BIT,
      pdTRUE,
      pdFALSE,
      xDelay);

    if (deviceConnected) {
      PostureType p;
      ActivityType a;

      // 1. Acquire Mutex and safely copy ALL required global data
      if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
        // Remote Data (HR, SpO2)
        dataToSend.hr = globalHR;
        dataToSend.o2 = globalSpO2;
        dataToSend.sos = globalSOS;

        // Local Data (Steps, GPS)
        dataToSend.steps = globalStepCount;
        dataToSend.lat = globalLatitude;
        dataToSend.lon = globalLongitude;
        dataToSend.speed = globalSpeed;

        // MPU Flags (Shake/Impact, Still/Posture)
        dataToSend.impact = globalImpact;
        dataToSend.still = globalStill;
        p = globalPosture;
        a = globalActivity;
        xSemaphoreGive(xDataMutex);
      }

      // 2. Clear and Populate the JSON document with ALL data points
      doc.clear();

      doc["HR"] = dataToSend.hr;
      doc["O2"] = dataToSend.o2;
      doc["ST"] = dataToSend.steps;
      doc["BAT"] = dataToSend.bat;

      doc["POSTURE"] = postureToString(p);
      doc["ACTIVITY"] = activityToString(a);

      JsonObject gpsJson = doc.createNestedObject("GPS");
      gpsJson["lat"] = dataToSend.lat;
      gpsJson["lon"] = dataToSend.lon;
      gpsJson["speed"] = dataToSend.speed;

      String jsonString;
      serializeJson(doc, jsonString);

      pTxCharacteristic->setValue(
        (uint8_t *)jsonString.c_str(),
        jsonString.length());
      pTxCharacteristic->notify();

      vTaskDelay(pdMS_TO_TICKS(1000));  // 🔑 BLE stability
    }

    else {
      
    }
  }
}



// --- TASK 6: Heart rate and Blood oxygen ---
void TaskMAX30102(void *pvParameters) {
  xSemaphoreTake(xI2CMutex, portMAX_DELAY);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    xSemaphoreGive(xI2CMutex);
    vTaskDelete(NULL);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  xSemaphoreGive(xI2CMutex);

  for (;;) {

    // ---------- Check sensor ----------
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    particleSensor.check();
    if (!particleSensor.available()) {
      xSemaphoreGive(xI2CMutex);
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    long irValue = particleSensor.getIR();
    particleSensor.nextSample();
    xSemaphoreGive(xI2CMutex);

    // ---------- Finger detection ----------
    if (irValue < 50000) {
      // No finger: keep last valid values
      if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
        globalHR = lastValidHR;
        globalSpO2 = lastValidSpO2;
        xSemaphoreGive(xDataMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // ---------- Pause BLE notifications while measuring ----------
    xEventGroupClearBits(bluetoothEvents, BLUETOOTH_SEND_NOW_BIT);

    // ---------- Collect 100 samples ----------
    for (int i = 0; i < 100; i++) {
      while (!particleSensor.available()) {
        particleSensor.check();
        vTaskDelay(pdMS_TO_TICKS(5));
      }

      xSemaphoreTake(xI2CMutex, portMAX_DELAY);
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();
      xSemaphoreGive(xI2CMutex);
    }

    // ---------- Calculate HR & SpO2 ----------
    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, 100,
      redBuffer,
      &spo2, &validSPO2,
      &heartRate, &validHeartRate);

    // ---------- Smooth & Validate ----------
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {

      // -------- HEART RATE --------
      if (validHeartRate && heartRate >= HR_MIN_VALID && heartRate <= HR_MAX_VALID) {
        lastValidHR = getSmoothedHR(heartRate);
        globalHR = lastValidHR;
      } else {
        globalHR = lastValidHR;  // keep last valid
      }

      // -------- SpO2 --------
      if (validSPO2 && spo2 >= SPO2_MIN_VALID && spo2 <= SPO2_MAX_VALID) {
        lastValidSpO2 = getSmoothedSpO2(spo2);
        globalSpO2 = lastValidSpO2;
      } else {
        globalSpO2 = lastValidSpO2;  // keep last valid
      }

      // -------- SOS AUTOMATIC BASED ON HR / SpO2 --------
      bool autoSOS = false;
      if (globalHR < 65 || globalHR > 150) autoSOS = true;
      if (globalSpO2 < 92) autoSOS = true;

      // Combine with BackButton SOS
      if (digitalRead(BACK_BUTTON_PIN) == LOW) {
        globalSOS = true;  // button pressed overrides
        drawSOSScreen();
        vibrateMotor(500);  // short vibration for connection
      } else {
        globalSOS = autoSOS;  // otherwise use autoSOS
      }

      xSemaphoreGive(xDataMutex);
    }

    // ---------- Resume BLE notifications ----------
    xEventGroupSetBits(bluetoothEvents, BLUETOOTH_SEND_NOW_BIT);

    // ---------- Wait before next measurement ----------
    vTaskDelay(pdMS_TO_TICKS(2000));  // smooth 2 sec interval
  }
}

// --- TASK 7: Back Button ---
void TaskBackButton(void *pvParameters) {
  bool buttonPressed = false;
  bool longPressActive = false;
  unsigned long pressStartTime = 0;

  for (;;) {

    bool buttonState = (digitalRead(BACK_BUTTON_PIN) == LOW);  // Active LOW

    // -------- Button just pressed --------
    if (buttonState && !buttonPressed) {
      buttonPressed = true;
      longPressActive = false;
      pressStartTime = millis();
    }

    // -------- Button held --------
    else if (buttonState && buttonPressed) {

      if (!longPressActive && (millis() - pressStartTime >= BACK_BUTTON_HOLD_TIME_MS)) {

        // 🚨 SOS ACTIVE
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
          globalSOS = true;
          vibrateMotor(2000);  // long vibration for warning
          drawSOSScreen();
          xSemaphoreGive(xDataMutex);
        }

        // BLE send immediately
        xEventGroupSetBits(bluetoothEvents, BLUETOOTH_SEND_NOW_BIT);

        longPressActive = true;
      }
    }

    // -------- Button released --------
    else if (!buttonState && buttonPressed) {

      // ⬅️ Short press → Back Screen
      if (!longPressActive) {
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
          if (currentScreen == 0)
            currentScreen = (ScreenType)(SCREEN_COUNT - 1);
          else
            currentScreen = (ScreenType)(currentScreen - 1);

          screenChanged = true;
          xSemaphoreGive(xDataMutex);
        }
      }

      // 🔄 Reset SOS on release
      if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
        globalSOS = false;
        xSemaphoreGive(xDataMutex);
      }

      buttonPressed = false;
      longPressActive = false;
    }

    // -------- Button idle (safety) --------
    if (!buttonState && !buttonPressed) {
      if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
        globalSOS = false;
        xSemaphoreGive(xDataMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // debounce + CPU friendly
  }
}

// --- TASK 8: Forward Button ---
void TaskForwardButton(void *pvParameters) {

  bool buttonPressed = false;

  for (;;) {

    bool buttonState = (digitalRead(FORWARD_BUTTON_PIN) == LOW);  // Active LOW

    // -------- Button just pressed --------
    if (buttonState && !buttonPressed) {
      buttonPressed = true;
    }

    // -------- Button released --------
    else if (!buttonState && buttonPressed) {

      if (xSemaphoreTake(xDataMutex, portMAX_DELAY)) {
        currentScreen = (ScreenType)((currentScreen + 1) % SCREEN_COUNT);
        screenChanged = true;
        xSemaphoreGive(xDataMutex);
      }

      buttonPressed = false;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --- TASK 9: Display ---
void TaskDisplay(void *pvParameters) {

  ScreenType lastScreen = SCREEN_COUNT;

  for (;;) {

    // ===== MED ALERT OVERLAY (TOP PRIORITY) =====
    if (medAlertActive) {

      drawMedicineAlertOverlay();

      if (millis() - medAlertStartTime >= MED_ALERT_DURATION_MS) {
        medAlertActive = false;
        screenChanged = true;  // force redraw
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;  // block ALL other screens
    }

    // ===== FIND MY WATCH OVERLAY =====
    if (globalFindMyWatch) {
      drawFindMyWatchScreen();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // ===== NORMAL UI =====
    if (screenChanged || currentScreen != lastScreen) {

      tft.fillScreen(COLOR_BLACK);

      switch (currentScreen) {
        case SCREEN_CLOCK: drawClockScreen(); break;
        case SCREEN_HEART: updateHeartRateScreen(); break;
        case SCREEN_SPO2: updateOxygenScreen(); break;
        case SCREEN_STEPS: drawStepCounterScreen(); break;
        case SCREEN_SPEED: drawSpeedScreen(); break;
      }

      lastScreen = currentScreen;
      screenChanged = false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// --- TASK 10: Find My Watch ---
void TaskFindMyWatch(void *pvParameters) {

  bool ledState = false;
  unsigned long findStartTime = 0;

  for (;;) {

    if (globalFindMyWatch) {
      if (findStartTime == 0) findStartTime = millis();

      if (millis() - findStartTime > 30000) {  // 30 seconds
        globalFindMyWatch = false;
        findStartTime = 0;
      } else {
        findStartTime = 0;
        vTaskDelay(pdMS_TO_TICKS(300));
      }

      // Strong repeating vibration
      digitalWrite(VIBRATION_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(400));
      digitalWrite(VIBRATION_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(200));

      // Flashing alert screen
      ledState = !ledState;

      uint16_t bg = ledState ? COLOR_CYAN : COLOR_BLACK;
      uint16_t fg = ledState ? COLOR_BLACK : COLOR_CYAN;

      tft.fillScreen(bg);
      tft.setFont(&FreeSansBold24pt7b);
      centerText("FIND ME", 120, fg, &FreeSansBold24pt7b);
      tft.setFont(&FreeSans9pt7b);
      centerText("Tap phone to stop", 160, fg, &FreeSans9pt7b);

    } else {
      vTaskDelay(pdMS_TO_TICKS(300));
    }
  }
}
/* ============================================================================================== */
/* ----------------------------------------- Screens -------------------------------------------- */
/* ============================================================================================== */
// ==========================================
// SCREEN 1: CLOCK
// ==========================================
void drawClockScreen() {

  if (!gpsTimeValid) {
    tft.fillScreen(COLOR_BLACK);
    centerText("WAITING FOR GPS", 130, COLOR_RED, &FreeSansBold12pt7b);
    return;
  }

  if (gpsTime.tm_min != lastMinute) {
    tft.fillScreen(COLOR_BLACK);

    drawArc(120, 120, 130, 230, 116, 119, COLOR_GREY);
    drawArc(120, 120, 310, 410, 116, 119, COLOR_GREY);

    int hour12 = gpsTime.tm_hour % 12;
    if (hour12 == 0) hour12 = 12;

    char hourStr[5], minStr[5], dateStr[30], battStr[10];

    sprintf(hourStr, "%d", hour12);
    sprintf(minStr, "%02d", gpsTime.tm_min);
    strftime(dateStr, 30, "%d/%m/%y", &gpsTime);

    for (int i = 0; dateStr[i]; i++) dateStr[i] = toupper(dateStr[i]);

    centerText(hourStr, 105, COLOR_WHITE, &FreeSansBold24pt7b);
    tft.fillRect(90, 120, 60, 2, COLOR_GREY);
    centerText(minStr, 170, COLOR_CYAN, &FreeSansBold24pt7b);

    centerText(dateStr, 50, COLOR_LIGHTGREY, &FreeSansBold12pt7b);

    lastMinute = gpsTime.tm_min;
  }
}

// ==========================================
// SCREEN 2: Heart rate
// ==========================================
void updateHeartRateScreen() {

  int hr;
  xSemaphoreTake(xDataMutex, portMAX_DELAY);
  hr = globalHR;
  xSemaphoreGive(xDataMutex);

  tft.fillScreen(COLOR_BLACK);
  drawHeart(120, 100, 50, COLOR_RED);

  if (hr <= 0) {
    centerText("Place Finger", 180, COLOR_WHITE, &FreeSansBold12pt7b);
  } else {
    char bpmStr[12];
    sprintf(bpmStr, "%d BPM", hr);
    centerText(bpmStr, 180, COLOR_GOLD, &FreeSansBold24pt7b);
  }
}


// ==========================================
// SCREEN 3: Blood Oxygen
// ==========================================
void updateOxygenScreen() {

  int o2;
  xSemaphoreTake(xDataMutex, portMAX_DELAY);
  o2 = globalSpO2;
  xSemaphoreGive(xDataMutex);

  tft.fillScreen(COLOR_BLACK);
  drawDrop(120, 100, 40, COLOR_CYAN);

  if (o2 <= 0) {
    centerText("Place Finger", 180, COLOR_WHITE, &FreeSansBold12pt7b);
  } else {
    char o2Str[10];
    sprintf(o2Str, "%d%%", o2);
    centerText(o2Str, 180, COLOR_GOLD, &FreeSansBold24pt7b);
    centerText("SpO2", 215, COLOR_LIGHTGREY, &FreeSans9pt7b);
  }
}

// ==========================================
// SCREEN 4: Step Counter
// ==========================================
void drawStepCounterScreen() {
  int steps;
  xSemaphoreTake(xDataMutex, portMAX_DELAY);
  steps = globalStepCount;
  xSemaphoreGive(xDataMutex);

  if (displayedSteps != steps) {
    tft.fillScreen(COLOR_BLACK);
    float percentage = (float)steps / (float)stepGoal;
    if (percentage > 1.0) percentage = 1.0;
    int endAngle = (int)(percentage * 360);
    drawArc(120, 120, 0, 360, 90, 96, COLOR_GREY);
    drawArc(120, 120, 270, 270 + endAngle, 90, 96, COLOR_WHITE);
    char stepStr[10];
    sprintf(stepStr, "%d", steps);
    centerText(stepStr, 150, COLOR_CYAN, &FreeSansBold24pt7b);
    char goalStr[20];
    sprintf(goalStr, "Goal: %d", stepGoal);
    centerText(goalStr, 175, COLOR_LIGHTGREY, &FreeSans9pt7b);
    displayedSteps = steps;
  }
}

// ==========================================
// SCREEN 5: Speed
// ==========================================
void drawSpeedScreen() {

  tft.fillScreen(COLOR_BLACK);

  tft.setFont(&FreeSansBold12pt7b);
  centerText("SPEED", 40, COLOR_GOLD, &FreeSansBold12pt7b);

  char speedStr[10];
  snprintf(speedStr, sizeof(speedStr), "%.1f km/h", globalSpeed);

  tft.setFont(&FreeSansBold24pt7b);
  centerText(speedStr, 130, COLOR_CYAN, &FreeSansBold24pt7b);
}

// ==========================================
// SOS SCREEN (Alert)
// ==========================================
void drawSOSScreen() {
  if (millis() - lastSosBlink > 250) {
    sosRedState = !sosRedState;
    lastSosBlink = millis();

    uint16_t bg = sosRedState ? COLOR_RED : COLOR_BLACK;
    uint16_t fg = sosRedState ? COLOR_WHITE : COLOR_RED;

    tft.fillScreen(bg);
    tft.fillTriangle(120, 40, 60, 140, 180, 140, fg);
    tft.fillRect(115, 65, 10, 45, bg);
    tft.fillCircle(120, 125, 5, bg);

    tft.setFont(&FreeSansBold24pt7b);
    centerText("SOS", 200, fg, &FreeSansBold24pt7b);

    tft.setFont(&FreeSans9pt7b);
    centerText("ALERT", 230, fg, &FreeSans9pt7b);
  }
}
// ==========================================
// Find My Watch ALERT OVERLAY
// ==========================================
void drawFindMyWatchScreen() {

  static bool drawn = false;
  if (!globalFindMyWatch) {
    drawn = false;
    return;
  }
  if (drawn) return;

  drawn = true;

  tft.fillScreen(COLOR_BLACK);

  // 🔍 Icon
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
  tft.setCursor(30, 40);
  tft.println("FIND MY WATCH");

  tft.setTextSize(1);
  tft.setCursor(45, 80);
  tft.println("Searching...");
}

// ==========================================
// MEDICINE ALERT OVERLAY
// ==========================================
void drawMedicineAlertOverlay() {

  static bool blink = false;
  static unsigned long lastBlink = 0;

  if (millis() - lastBlink > 300) {
    blink = !blink;
    lastBlink = millis();
  }

  uint16_t bg = blink ? COLOR_CYAN : COLOR_BLACK;
  uint16_t fg = blink ? COLOR_WHITE : COLOR_CYAN;

  tft.fillScreen(bg);

  // Pill icon
  tft.fillRoundRect(70, 70, 100, 50, 25, fg);
  tft.fillRoundRect(70, 70, 50, 50, 25, bg);
  tft.drawLine(120, 70, 120, 120, COLOR_BLACK);

  tft.setFont(&FreeSansBold12pt7b);
  centerText("MEDICINE", 160, fg, &FreeSansBold12pt7b);

  tft.setFont(&FreeSans9pt7b);
  centerText("Time to take pill", 190, fg, &FreeSans9pt7b);
}

/* ============================================================================================== */
/* ------------------------------------- Helper Function ---------------------------------------- */
/* ============================================================================================== */
// ================== Center Text ====================
void centerText(const char *text, int y, uint16_t color, const GFXfont *font) {
  tft.setFont(font);
  tft.setTextColor(color);
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  tft.setCursor((240 - w) / 2, y);
  tft.print(text);
}
// ================== draw simple Arc ====================
void drawArc(int x, int y, int startAngle, int endAngle, int r1, int r2, uint16_t color) {
  for (int r = r1; r <= r2; r++) {
    for (int a = startAngle; a <= endAngle; a++) {
      float rad = a * DEG_TO_RAD;
      int px = x + cos(rad) * r;
      int py = y + sin(rad) * r;
      tft.drawPixel(px, py, color);
    }
  }
}
// ================== draw Heart ====================
void drawHeart(int x, int y, int size, uint16_t color) {
  int r = size / 2;
  int x_offset = r;
  int y_offset = r / 2;
  tft.fillCircle(x - x_offset + 2, y - y_offset, r, color);
  tft.fillCircle(x + x_offset - 2, y - y_offset, r, color);
  tft.fillTriangle(x - size, y - y_offset, x + size, y - y_offset, x, y + size * 1.2, color);
}
// ================== draw drop ====================
void drawDrop(int x, int y, int size, uint16_t color) {
  tft.fillCircle(x, y, size, color);
  tft.fillTriangle(x - size + 2, y, x + size - 2, y, x, y - 2 * size, color);
  if (color != COLOR_BLACK && color != COLOR_GREY) {
    tft.fillCircle(x - 5, y + 5, 4, COLOR_WHITE);
    tft.fillCircle(x + 6, y - 2, 3, COLOR_WHITE);
  }
}
// ==================SpO2====================
int getSmoothedSpO2(int newValue) {
  spo2History[spo2Index++] = newValue;

  if (spo2Index >= SPO2_AVG_WINDOW) {
    spo2Index = 0;
    spo2BufferFull = true;
  }

  int count = spo2BufferFull ? SPO2_AVG_WINDOW : spo2Index;
  int sum = 0;

  for (int i = 0; i < count; i++)
    sum += spo2History[i];

  return sum / count;
}
// ============== Heart Rate ==================
int getSmoothedHR(int newValue) {
  hrHistory[hrIndex++] = newValue;

  if (hrIndex >= HR_AVG_WINDOW) {
    hrIndex = 0;
    hrBufferFull = true;
  }

  int count = hrBufferFull ? HR_AVG_WINDOW : hrIndex;
  int sum = 0;

  for (int i = 0; i < count; i++)
    sum += hrHistory[i];

  return sum / count;
}
// ============== vibrate Motor ==================
void vibrateMotor(int duration_ms) {
  digitalWrite(VIBRATION_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(duration_ms));
  digitalWrite(VIBRATION_PIN, LOW);
}
// ============== SITTING or STANDING ==================
String postureToString(PostureType p) {
  if (p == POSTURE_SITTING) return "SITTING";
  if (p == POSTURE_STANDING) return "STANDING";
  return "UNKNOWN";
}
// ============== SITTING or STANDING ==================
String activityToString(ActivityType a) {
  if (a == ACTIVITY_WALKING) return "WALKING";
  if (a == ACTIVITY_RUNNING) return "RUNNING";
  return "UNKNOWN";
}