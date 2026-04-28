/*
 * ESP32C3 GP02, MPU6050, Battery, UART, and BLE Monitor - RTOS Version
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
#include "spo2_algorithm.h"

// --- BLE Libraries ---
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

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
// Battery Monitor
#define ANALOG_IN_PIN 7
#define REF_VOLTAGE 3.3
#define ADC_RESOLUTION 4095.0
#define R1 30000.0
#define R2 7500.0
#define BATTERY_MAX_VOLTAGE 3.5
#define BATTERY_MIN_VOLTAGE 2.0

// Fall Detection Constants
const unsigned long FALL_DETECTION_TIMEOUT_MS = 20000;

// ------------ Objects ------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
HardwareSerial otherSerial(2);
MPU6050 mpu;
MAX30105 particleSensor;


// ------------ FreeRTOS Handles & Events ------------
SemaphoreHandle_t xDataMutex;
SemaphoreHandle_t xSerialMutex;
EventGroupHandle_t bluetoothEvents;
SemaphoreHandle_t xI2CMutex;
#define BLUETOOTH_SEND_NOW_BIT (1 << 0)

// ------------ Globals (Shared Data) ------------
// Local Data (Collected & Sent via BLE)
volatile int globalStepCount = 0;
volatile float globalBatteryPct = 0.0;
volatile double globalLatitude = 0.0;
volatile double globalLongitude = 0.0;
volatile float globalSpeed = 0.0;

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

// Heart rate and oxygen
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;
enum MaxState {
  STATE_WAITING_FINGER,
  STATE_MEASURING,
  STATE_LOCKED
};
// heart rate
#define HR_MIN_VALID 40
#define HR_MAX_VALID 180
#define HR_AVG_WINDOW 5
int hrHistory[HR_AVG_WINDOW];
uint8_t hrIndex = 0;
bool hrBufferFull = false;

int lastValidHR = 0;

// Blood oxygen
MaxState currentState = STATE_WAITING_FINGER;
bool bpmLocked = false;
#define SPO2_MIN_VALID 90
#define SPO2_MAX_VALID 100
#define SPO2_AVG_WINDOW 6

int spo2History[SPO2_AVG_WINDOW];
uint8_t spo2Index = 0;
bool spo2BufferFull = false;
int lastValidSpO2 = 0;

/************ Buttons ************/
// Back Button
#define BACK_BUTTON_PIN 3
#define BACK_BUTTON_HOLD_TIME_MS 3000
volatile bool backButtonPressed = false;

/************ Vibration motor ************/
#define VIBRATION_PIN 5
// Constants
const float motionThreshold = 0.15;
const float shakeThreshold = 0.5;
const unsigned long stillDelay = 3000;

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
    if (rxValue.length() > 0) {
      xSemaphoreTake(xSerialMutex, portMAX_DELAY);
      Serial.print("[BLE RX] Received: ");
      Serial.println(rxValue);
      xSemaphoreGive(xSerialMutex);
    }
  }
};

// ------------ Function Prototypes ------------
// All sensor/data acquisition functions are implemented as FreeRTOS Tasks
void TaskGPS(void *pvParameters);
void TaskMPU(void *pvParameters);
void TaskBattery(void *pvParameters);
void TaskUARTControl(void *pvParameters);
void TaskUARTReceive(void *pvParameters);
void TaskBLESend(void *pvParameters);
void TaskSerialControl(void *pvParameters);
void TaskMAX30102(void *pvParameters);
void TaskBackButton(void *pvParameters);


// Helper function
int getSmoothedHR(int newValue);
int getSmoothedSpO2(int newValue);
void vibrateMotor(int duration_ms);

// ----------------------------------------------------
// MAIN SETUP AND LOOP
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);

  // 1. Initialize Hardware (MPU, GPS, BackButton , ForwardButton, Vibration motor)
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Wire.begin(8, 9);
  mpu.initialize();
  pinMode(BACK_BUTTON_PIN, INPUT_PULLUP);
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);  // OFF initially

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
  xTaskCreate(TaskGPS, "GPS_Task", 4096, NULL, 1, NULL);
  xTaskCreate(TaskMPU, "MPU_Task", 4096, NULL, 2, NULL);
  xTaskCreate(TaskBattery, "Bat_Task", 2048, NULL, 1, NULL);
  xTaskCreate(TaskMAX30102, "MAX30102_Task", 8192, NULL, 1, NULL);
  xTaskCreate(TaskBackButton, "Back_Button", 2048, NULL, 2, NULL);  // Higher Priority for emergency

  xTaskCreate(TaskBLESend, "BLE_Send", 8192, NULL, 2, NULL);

  xTaskCreate(TaskSerialControl, "UI_Task", 4096, NULL, 1, NULL);
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
        if (gps.location.isValid() && gps.location.isUpdated()) {
          if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            globalLatitude = gps.location.lat();
            globalLongitude = gps.location.lng();
            if (gps.speed.isValid()) {
              globalSpeed = gps.speed.kmph();
            } else {
              globalSpeed = 0.0;
            }
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

    // Step Count Logic
    float diff = accZ - prevAz;
    if (diff > 0.25 && (now - lastStepTime) > 300) {
      lastStepTime = now;
      if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
        globalStepCount++;
        xSemaphoreGive(xDataMutex);
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
      Serial.printf("[SENSOR OK] Steps: %d, Batt: %.1f%% (BLE Connected: %s)\n",
                    globalStepCount, globalBatteryPct, deviceConnected ? "YES" : "NO");
      xSemaphoreGive(xSerialMutex);
    }

    prevAx = accX;
    prevAy = accY;
    prevAz = accZ;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


// --- TASK 3: Battery Monitor (INDEPENDENT) ---
void TaskBattery(void *pvParameters) {
  for (;;) {
    int adc_value = analogRead(ANALOG_IN_PIN);
    float voltage_adc = (adc_value * REF_VOLTAGE) / ADC_RESOLUTION;
    float battery_voltage = voltage_adc * (R1 + R2) / R2;

    float pct = (battery_voltage - BATTERY_MIN_VOLTAGE) * 100.0 / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);

    if (pct > 100) pct = 100;
    if (pct < 0) pct = 0;
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
      globalBatteryPct = pct;
      xSemaphoreGive(xDataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}


// --- TASK 4: BLE Send (SENDS ALL REQUIRED DATA) ---
void TaskBLESend(void *pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(5000);
  DynamicJsonDocument doc(JSON_DOC_SIZE);

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

      // 1. Acquire Mutex and safely copy ALL required global data
      if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
        // Remote Data (HR, SpO2)
        dataToSend.hr = globalHR;
        dataToSend.o2 = globalSpO2;
        dataToSend.sos = globalSOS;

        // Local Data (Steps, GPS, Battery)
        dataToSend.steps = globalStepCount;
        dataToSend.lat = globalLatitude;
        dataToSend.lon = globalLongitude;
        dataToSend.speed = globalSpeed;
        dataToSend.bat = globalBatteryPct;

        // MPU Flags (Shake/Impact, Still/Posture)
        dataToSend.impact = globalImpact;
        dataToSend.still = globalStill;

        xSemaphoreGive(xDataMutex);
      }

      // 2. Clear and Populate the JSON document with ALL data points
      doc.clear();

      // Health Data
      doc["HR"] = dataToSend.hr;
      doc["O2"] = dataToSend.o2;

      // Motion/Steps/Battery
      doc["ST"] = dataToSend.steps;
      doc["BAT"] = dataToSend.bat;

      // Emergency/Status Flags (SOS, Shake, Still)
      doc["SOS"] = dataToSend.sos || (dataToSend.impact && dataToSend.still);  // Composite SOS
      doc["SHAKE"] = dataToSend.impact;                                        // SHAKE is represented by IMPACT (IMP)
      doc["POSTURE"] = dataToSend.still;                                       // STILL indicates a possible Man-Down posture

      // Nested GPS object
      JsonObject gpsJson = doc.createNestedObject("GPS");
      gpsJson["lat"] = dataToSend.lat;
      gpsJson["lon"] = dataToSend.lon;
      gpsJson["speed"] = dataToSend.speed;

      // 3. Serialize and Send the JSON string via BLE (Notify)
      String jsonString;
      serializeJson(doc, jsonString);

      pTxCharacteristic->setValue((uint8_t *)jsonString.c_str(), jsonString.length());
      pTxCharacteristic->notify();

      xSemaphoreTake(xSerialMutex, portMAX_DELAY);
      Serial.print("[BLE TX] Notifying JSON: ");
      Serial.println(jsonString);
      xSemaphoreGive(xSerialMutex);
    } else {
      vibrateMotor(1000);  // short vibration for connection
    }
  }
}

// --- TASK 5: Serial Control (UI/Debug) ---
void TaskSerialControl(void *pvParameters) {
  for (;;) {
    if (Serial.available()) {
      char cmd = Serial.read();
      xSemaphoreTake(xSerialMutex, portMAX_DELAY);

      if (cmd == '1') {  // Local Data Request
        Serial.println("--- GPS/MPU/Battery STATUS ---");
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
          Serial.printf("Lat: %.6f, Lng: %.6f, Speed: %.2f km/h\n", globalLatitude, globalLongitude, globalSpeed);
          Serial.printf("Steps: %d, Battery: %.1f%%\n", globalStepCount, globalBatteryPct);
          Serial.printf("Impact (Shake): %s, Still (Posture): %s\n", globalImpact ? "YES" : "NO", globalStill ? "YES" : "NO");
          xSemaphoreGive(xDataMutex);
        }
      } else if (cmd == '4') {  // Remote Data Status
        Serial.println("--- REMOTE DATA (via UART) ---");
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
          Serial.printf("HR: %d, SpO2: %d, Temp: %d, SOS: %s\n",
                        globalHR, globalSpO2, globalTemp, globalSOS ? "ACTIVE" : "OFF");
          xSemaphoreGive(xDataMutex);
        }
      } else {
        Serial.println("Invalid command. Use '1' for Local Data, '4' for Remote Data.");
      }

      xSemaphoreGive(xSerialMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --- TASK 5: Heart rate and Blood oxygen ---
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
        globalSOS = true;   // button pressed overrides
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

// --- TASK 6: Back Button ---
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
          xSemaphoreGive(xDataMutex);
        }

        // BLE send immediately
        xEventGroupSetBits(bluetoothEvents, BLUETOOTH_SEND_NOW_BIT);

        longPressActive = true;
      }
    }

    // -------- Button released --------
    else if (!buttonState && buttonPressed) {

      // ⬅️ Short press → Back
      if (!longPressActive) {
        backButtonPressed = true;
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


// ==========================================
// HELPER FUNCTION
// ==========================================
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
// ==============Heart Rate==================
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
// ==============vibrate Motor==================
// Function to trigger vibration for a short duration
void vibrateMotor(int duration_ms) {
  digitalWrite(VIBRATION_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(duration_ms));
  digitalWrite(VIBRATION_PIN, LOW);
}