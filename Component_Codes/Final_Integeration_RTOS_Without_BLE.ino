/*
 * ESP32C3 GP02, MPU6050, and Battery Monitor - RTOS Version
 * @author : Ziad-Elmekawy (Modified for RTOS)
 * @date   : 28, November, 2025
 */

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>

// ------------ Configurations ------------
#define GPS_RX 20  
#define GPS_TX 21  
#define ANALOG_IN_PIN 1      
#define REF_VOLTAGE    3.3
#define ADC_RESOLUTION 4095.0
#define R1 30000.0
#define R2 7500.0
#define BATTERY_MAX_VOLTAGE 3.5
#define BATTERY_MIN_VOLTAGE 2.0

// ------------ Objects ------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
MPU6050 mpu;

// ------------ Globals (Shared Resources) ------------
// We use a Mutex to protect variables shared between tasks
SemaphoreHandle_t xDataMutex;   // Protects variable access
SemaphoreHandle_t xSerialMutex; // Protects Serial printing

volatile int globalStepCount = 0;
volatile float globalBatteryPct = 0.0;

// MPU Variables maintained internally by MPU Task
float prevAx = 0, prevAy = 0, prevAz = 0;
unsigned long lastMotionTime = 0;
unsigned long lastStepTime = 0;
unsigned char screen_status = 'n';

// Constants
const float motionThreshold = 0.15;   
const float shakeThreshold  = 0.5;    
const unsigned long stillDelay = 3000; 

// ------------ Function Prototypes ------------
void TaskGPS(void *pvParameters);
void TaskMPU(void *pvParameters);
void TaskBattery(void *pvParameters);
void TaskSerialControl(void *pvParameters);

void setup() {
  // 1. Init Serial and Hardware
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Wire.begin(8, 9);
  mpu.initialize();

  Serial.println("--- ESP32-C3 RTOS System Starting ---");

  // 2. Create Mutexes
  xDataMutex = xSemaphoreCreateMutex();
  xSerialMutex = xSemaphoreCreateMutex();

  // 3. Create Tasks
  // Core 0 is generally used for WiFi, but C3 is single core, so core ID is moot.
  
  // GPS Task: High stack needed for parsing, normal priority
  xTaskCreate(TaskGPS, "GPS_Task", 4096, NULL, 1, NULL);

  // MPU Task: Critical for step counting, High priority
  xTaskCreate(TaskMPU, "MPU_Task", 4096, NULL, 2, NULL);

  // Battery Task: Low priority, runs rarely
  xTaskCreate(TaskBattery, "Bat_Task", 2048, NULL, 1, NULL);

  // Serial/Control Task: Handles user input
  xTaskCreate(TaskSerialControl, "UI_Task", 4096, NULL, 1, NULL);
}

void loop() {
  // In RTOS, the loop is empty or used for watchdog feeding.
  // Everything happens in Tasks.
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
}

/* ================================================================ */
/* ----------------------- RTOS TASKS ----------------------------- */
/* ================================================================ */

// TASK 1: Handle GPS Parsing continuously
void TaskGPS(void *pvParameters) {
  for (;;) {
    // Read all available characters from GPS
    while (gpsSerial.available() > 0) {
      // We lock DataMutex briefly if we were writing to shared vars, 
      // but TinyGPS internal struct is complex. 
      // For this example, we protect the READ operation in the Output task instead.
      gps.encode(gpsSerial.read());
    }
    // Yield to other tasks for 10ms
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// TASK 2: Handle MPU logic (Screen, Steps, Motion)
void TaskMPU(void *pvParameters) {
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;

  for (;;) {
    // 1. Read Raw Data ONCE per cycle (Efficient)
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    float accX = ax_raw / 16384.0;
    float accY = ay_raw / 16384.0;
    float accZ = az_raw / 16384.0;

    // 2. Screen Logic
    float roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * 180 / PI;
    
    // Check Screen ON/OFF
    // Use Serial Mutex to print
    if ((roll >= -50) && (roll <= 15) && screen_status == 'f') {
      xSemaphoreTake(xSerialMutex, portMAX_DELAY);
      Serial.println("Screen ON");
      xSemaphoreGive(xSerialMutex);
      screen_status = 'n';
    } 
    else if ((roll > 15 || roll < -50) && screen_status == 'n') {
      xSemaphoreTake(xSerialMutex, portMAX_DELAY);
      Serial.println("Screen OFF");
      xSemaphoreGive(xSerialMutex);
      screen_status = 'f';
    }

    // 3. Step Count Logic
    float diff = accZ - prevAz;
    unsigned long now = millis();
    
    if (diff > 0.25 && (now - lastStepTime) > 300) {
      lastStepTime = now;
      
      // Protect shared variable write
      xSemaphoreTake(xDataMutex, portMAX_DELAY);
      globalStepCount++;
      int currentSteps = globalStepCount; // Local copy for printing
      xSemaphoreGive(xDataMutex);

      xSemaphoreTake(xSerialMutex, portMAX_DELAY);
      Serial.print("Step detected! Total = ");
      Serial.println(currentSteps);
      xSemaphoreGive(xSerialMutex);
    }

    // 4. Motion/Still Logic
    float dX = abs(accX - prevAx);
    float dY = abs(accY - prevAy);
    float dZ = abs(accZ - prevAz);
    float delta = max(dX, max(dY, dZ));

    xSemaphoreTake(xSerialMutex, portMAX_DELAY);
    if (delta > motionThreshold && delta < shakeThreshold) {
      Serial.println("🏃 Motion detected!");
      lastMotionTime = millis();
    } else if (delta >= shakeThreshold) {
      Serial.println("⚠️ Shake/Impact!");
      lastMotionTime = millis();
    } else if (millis() - lastMotionTime > stillDelay) {
      // Only print "Still" once every few seconds to avoid spamming
      if ((millis() - lastMotionTime) < (stillDelay + 200)) { 
          Serial.println("🧘 Device is still."); 
      }
    }
    xSemaphoreGive(xSerialMutex);

    // Save history
    prevAx = accX; prevAy = accY; prevAz = accZ;

    // Run this task at 10Hz (every 100ms)
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// TASK 3: Battery Monitor
void TaskBattery(void *pvParameters) {
  for (;;) {
    int adc_value = analogRead(ANALOG_IN_PIN);
    float voltage_adc = (adc_value * REF_VOLTAGE) / ADC_RESOLUTION;
    float battery_voltage = voltage_adc * (R1 + R2) / R2;

    float pct = (battery_voltage - BATTERY_MIN_VOLTAGE) * 100.0 /
                (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);

    if (pct > 100) pct = 100;
    if (pct < 0) pct = 0;

    // Update shared variable safely
    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    globalBatteryPct = pct;
    xSemaphoreGive(xDataMutex);

    // Check battery every 2 seconds
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// TASK 4: User Input and Reporting
void TaskSerialControl(void *pvParameters) {
  for (;;) {
    if (Serial.available()) {
      char cmd = Serial.read();
      
      // Grab Serial Mutex before printing results
      xSemaphoreTake(xSerialMutex, portMAX_DELAY);

      if (cmd == '1') { // GPS Request
        Serial.println("--- GPS STATUS ---");
        if (gps.location.isValid()) {
           Serial.print("Lat: "); Serial.println(gps.location.lat(), 6);
           Serial.print("Lng: "); Serial.println(gps.location.lng(), 6);
        } else {
           Serial.println("GPS Searching...");
        }
      } 
      else if (cmd == '2') { // MPU Request
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        int steps = globalStepCount;
        xSemaphoreGive(xDataMutex);
        Serial.print("Current Steps: "); Serial.println(steps);
      } 
      else if (cmd == '3') { // Battery Request
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        float bat = globalBatteryPct;
        xSemaphoreGive(xDataMutex);
        Serial.print("Battery: "); Serial.print(bat, 1); Serial.println("%");
      }

      xSemaphoreGive(xSerialMutex);
    }
    // Check for input every 50ms
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}