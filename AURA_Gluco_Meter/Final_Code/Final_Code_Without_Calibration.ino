/*
 * ESP32-C3 RTOS Health Monitor with BLE
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C  // Most common SSD1306 address

// Pins
#define BUTTON_PIN 6
#define ONE_WIRE_BUS 3
#define BUZZER_PIN 2

// Tones
#define TONE_LOW_GLUCOSE     1000
#define TONE_HIGH_GLUCOSE    1500
#define TONE_MEASUREMENT     2000
#define TONE_SENSOR_ERROR     700
#define TONE_HIGH_TEMP       1200

// States
enum ScreenState {
  SPLASH_SCREEN,
  WAIT_SCREEN,
  MEASURING_GLUCOSE_SCREEN,
  GLUCOSE_SCREEN,
  MEASURING_TEMP_SCREEN,
  TEMP_SCREEN
};

volatile ScreenState currentScreen = WAIT_SCREEN;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_ADS1115 ads;

// Shared data
float glucoseValue = 0;
float tempValue = 0;
bool fingerDetected = false;
int buzzerTone = 0;

// Queue
QueueHandle_t buzzerQueue;

// ================= BLE =================

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

const size_t JSON_DOC_SIZE = 256;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      // Handle received data
    }
  }
};

void initBLE() {
  BLEDevice::init("AURA Gluco-Meter");
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
}

void sendBLEData(float glucose, float temperature) {
  if (deviceConnected) {
    DynamicJsonDocument doc(JSON_DOC_SIZE);
    doc["GLU"] = glucose;
    doc["TEMP"] = temperature;

    String jsonString;
    serializeJson(doc, jsonString);

    pTxCharacteristic->setValue((uint8_t*)jsonString.c_str(), jsonString.length());
    pTxCharacteristic->notify();
  }
}

// ================= DRAW HELPERS =================

void drawBattery(int x, int y, int percentage) {
  display.drawRect(x, y, 22, 10, SSD1306_WHITE);
  display.fillRect(x + 22, y + 3, 3, 4, SSD1306_WHITE);
  int fillWidth = map(percentage, 0, 100, 0, 20);
  display.fillRect(x + 1, y + 1, fillWidth, 8, SSD1306_WHITE);
}

void drawBluetooth(int x, int y, bool connected) {
  if (connected) {
    display.drawLine(x + 3, y,     x + 3, y + 12, SSD1306_WHITE);
    display.drawLine(x + 4, y,     x + 4, y + 12, SSD1306_WHITE);
    display.drawLine(x + 3, y,     x + 7, y + 3,  SSD1306_WHITE);
    display.drawLine(x + 4, y,     x + 8, y + 3,  SSD1306_WHITE);
    display.drawLine(x + 7, y + 3, x + 3, y + 6,  SSD1306_WHITE);
    display.drawLine(x + 8, y + 3, x + 4, y + 6,  SSD1306_WHITE);
    display.drawLine(x + 3, y + 6, x + 7, y + 9,  SSD1306_WHITE);
    display.drawLine(x + 4, y + 6, x + 8, y + 9,  SSD1306_WHITE);
    display.drawLine(x + 7, y + 9, x + 3, y + 12, SSD1306_WHITE);
    display.drawLine(x + 8, y + 9, x + 4, y + 12, SSD1306_WHITE);
    display.fillTriangle(x + 4, y + 1, x + 7, y + 3, x + 4, y + 6,  SSD1306_WHITE);
    display.fillTriangle(x + 4, y + 6, x + 7, y + 9, x + 4, y + 11, SSD1306_WHITE);
    display.fillRect(x - 3, y + 3, 2, 2, SSD1306_WHITE);
    display.fillRect(x + 9, y + 3, 2, 2, SSD1306_WHITE);
    display.fillRect(x - 3, y + 8, 2, 2, SSD1306_WHITE);
    display.fillRect(x + 9, y + 8, 2, 2, SSD1306_WHITE);
  } else {
    display.drawLine(x + 3, y,     x + 3, y + 12, SSD1306_WHITE);
    display.drawLine(x + 3, y,     x + 7, y + 3,  SSD1306_WHITE);
    display.drawLine(x + 7, y + 3, x + 3, y + 6,  SSD1306_WHITE);
    display.drawLine(x + 3, y + 6, x + 7, y + 9,  SSD1306_WHITE);
    display.drawLine(x + 7, y + 9, x + 3, y + 12, SSD1306_WHITE);
    display.drawPixel(x - 2, y + 4, SSD1306_WHITE);
    display.drawPixel(x + 9, y + 4, SSD1306_WHITE);
    display.drawPixel(x - 2, y + 9, SSD1306_WHITE);
    display.drawPixel(x + 9, y + 9, SSD1306_WHITE);
  }
}

// ================= BUZZER =================

void playTone(int frequency, int duration) {
  ledcWriteTone(BUZZER_PIN, frequency);
  vTaskDelay(duration / portTICK_PERIOD_MS);
  ledcWriteTone(BUZZER_PIN, 0);
}

void BuzzerTask(void *pvParameters) {
  int tone;
  while (1) {
    if (xQueueReceive(buzzerQueue, &tone, portMAX_DELAY)) {
      switch (tone) {
        case TONE_LOW_GLUCOSE:
          for(int i=0;i<3;i++){
            playTone(TONE_LOW_GLUCOSE, 200);
            vTaskDelay(150 / portTICK_PERIOD_MS);
          }
          break;

        case TONE_HIGH_GLUCOSE:
          playTone(TONE_HIGH_GLUCOSE, 700);
          break;

        case TONE_MEASUREMENT:
          for(int i=0;i<2;i++){
            playTone(TONE_MEASUREMENT, 150);
            vTaskDelay(100 / portTICK_PERIOD_MS);
          }
          break;

        case TONE_SENSOR_ERROR:
          for(int i=0;i<2;i++){
            playTone(TONE_SENSOR_ERROR, 300);
            vTaskDelay(150 / portTICK_PERIOD_MS);
          }
          break;

        case TONE_HIGH_TEMP:
          for(int i=0;i<2;i++){
            playTone(TONE_HIGH_TEMP, 500);
            vTaskDelay(200 / portTICK_PERIOD_MS);
          }
          break;
      }
    }
  }
}

// ================= SENSOR =================

float readGlucose() {
  long sum = 0;
  for (int i = 0; i < 70; i++) {
    sum += ads.readADC_SingleEnded(0);
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }

  float voltage = ads.computeVolts(sum / 70);
  float delta = 3.85 - voltage;

  if (delta < 0.01) {
    fingerDetected = false;
    int tone = TONE_SENSOR_ERROR;
    xQueueSend(buzzerQueue, &tone, 0);
    return 0;
  }

  fingerDetected = true;
  float glucose = (1000 * delta) + 70;

  int tone;
  if (glucose < 70)
    tone = TONE_LOW_GLUCOSE;
  else if (glucose > 180)
    tone = TONE_HIGH_GLUCOSE;
  else
    tone = TONE_MEASUREMENT;

  xQueueSend(buzzerQueue, &tone, 0);

  return glucose;
}

float readTemp() {
  float temp_value = 0.0 ;
  float t= 0.0 ;
  const int samples = 20;
  const float TEMP_OFFSET = 8.0;  // Adjust after calibration
  float total = 0;
  int validCount = 0;

  sensors.setResolution(12);
  sensors.setWaitForConversion(true);  // Auto-wait 750ms per read

  for (int i = 0; i < samples; i++) {
    sensors.requestTemperatures();
    t = sensors.getTempCByIndex(0);

    if (t != DEVICE_DISCONNECTED_C && t > 0.0 && t < 50.0) {
      total += t;
      validCount++;
    }
  }
  
  // Check if we got valid readings
  if (validCount == 0) {
    int tone = TONE_SENSOR_ERROR;
    xQueueSend(buzzerQueue, &tone, 0);
    return DEVICE_DISCONNECTED_C;
  }
  
  temp_value = (total / validCount) + TEMP_OFFSET ;

  if (temp_value > 38.0) {
    int tone = TONE_HIGH_TEMP;
    xQueueSend(buzzerQueue, &tone, 0);
  }

  return temp_value;
}

void SensorTask(void *pvParameters) {
  while (1) {
    // Read glucose when in MEASURING_GLUCOSE_SCREEN
    if (currentScreen == MEASURING_GLUCOSE_SCREEN) {
      glucoseValue = readGlucose();
      currentScreen = GLUCOSE_SCREEN;
    }
    // Read temperature when in MEASURING_TEMP_SCREEN
    else if (currentScreen == MEASURING_TEMP_SCREEN) {
      tempValue = readTemp();
      currentScreen = TEMP_SCREEN;
      // Send BLE data only after temperature reading
      sendBLEData(glucoseValue, tempValue);
      // Play tone after temperature measurement
      int tone = TONE_MEASUREMENT;
      xQueueSend(buzzerQueue, &tone, 0);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ================= DISPLAY =================

void showSplashScreen(int batteryPercent, bool btConnected) {
  display.clearDisplay();
  drawBluetooth(4, 1, btConnected);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  char batStr[5];
  snprintf(batStr, sizeof(batStr), "%d%%", batteryPercent);
  display.setCursor(batteryPercent == 100 ? 74 : 80, 3);
  display.println(batStr);
  drawBattery(100, 2, batteryPercent);
  display.setTextSize(3);
  display.setCursor(24, 18);
  display.println("AURA");
  display.setTextSize(1);
  display.setCursor(15, 54);
  display.println("Health Monitor");
  display.display();
}

void showPressButtonScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(8, 16);
  display.println("Press to");
  display.setCursor(28, 38);
  display.println("Start");
  display.display();
}

void showBloodGlucoseScreen(float glucose) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(22, 2);
  display.println("Blood Glucose");
  display.drawLine(0, 13, 128, 13, SSD1306_WHITE);
  display.setTextSize(3);
  char valStr[8];
  dtostrf(glucose, 4, 1, valStr);
  int valWidth = strlen(valStr) * 18;
  display.setCursor((128 - valWidth) / 2, 20);
  display.println(valStr);
  display.setTextSize(1);
  display.setCursor(34, 54);
  display.println("mg/dL");
  display.display();
}

void showTemperatureScreen(float temperature) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(22, 2);
  display.println("Body Temperature");
  display.drawLine(0, 13, 128, 13, SSD1306_WHITE);

  // Handle sensor error
  if (temperature == DEVICE_DISCONNECTED_C) {
    display.setTextSize(1);
    display.setCursor(10, 28);
    display.println("Sensor Error!");
    display.setCursor(10, 42);
    display.println("Check connection");
  } else {
    display.setTextSize(3);
    char valStr[8];
    dtostrf(temperature, 4, 1, valStr);
    int valWidth = strlen(valStr) * 18;
    display.setCursor((128 - valWidth) / 2, 20);
    display.println(valStr);
    display.setTextSize(1);
    display.setCursor(50, 54);
    display.println("\xF8""C");  // °C symbol
  }
  display.display();
}

void DisplayTask(void *pvParameters) {
  while (1) {
    display.clearDisplay();

    switch (currentScreen) {
      case SPLASH_SCREEN:
        showSplashScreen(70, deviceConnected);
        break;
        
      case WAIT_SCREEN:
        showPressButtonScreen();
        break;

      case MEASURING_GLUCOSE_SCREEN:
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(10,24);
        display.println("Measuring");
        display.display();
        break;

      case GLUCOSE_SCREEN:
        showBloodGlucoseScreen(glucoseValue);
        break;

      case MEASURING_TEMP_SCREEN:
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(10,24);
        display.println("Measuring");
        display.display();
        break;

      case TEMP_SCREEN:
        showTemperatureScreen(tempValue);
        break;
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// ================= BUTTON =================

void ButtonTask(void *pvParameters) {
  while (1) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      vTaskDelay(50 / portTICK_PERIOD_MS);

      while (digitalRead(BUTTON_PIN) == LOW)
        vTaskDelay(10 / portTICK_PERIOD_MS);

      // State machine for button presses
      if (currentScreen == SPLASH_SCREEN)
        currentScreen = WAIT_SCREEN;
      else if (currentScreen == WAIT_SCREEN)
        currentScreen = MEASURING_GLUCOSE_SCREEN;
      else if (currentScreen == GLUCOSE_SCREEN)
        currentScreen = MEASURING_TEMP_SCREEN;
      else if (currentScreen == TEMP_SCREEN)
        currentScreen = SPLASH_SCREEN;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// ================= SETUP =================

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(8, 9);  // ESP32-C3 I2C pins (SDA=8, SCL=9)
  Wire.setClock(100000);
  delay(200);  // Give I2C time to stabilize
  
  // Initialize DS18B20
  sensors.begin();
  
  ads.begin();

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    while (1);
  }
  
  // Clear and initialize display
  display.clearDisplay();
  display.display();
  delay(100);  // Brief delay for display to initialize

  // Start with splash screen
  currentScreen = SPLASH_SCREEN;

  ledcAttach(BUZZER_PIN, 2000, 8);

  buzzerQueue = xQueueCreate(5, sizeof(int));

  // Initialize BLE
  initBLE();

  xTaskCreate(ButtonTask, "Button Task", 2048, NULL, 1, NULL);
  xTaskCreate(DisplayTask, "Display Task", 8192, NULL, 1, NULL);
  xTaskCreate(SensorTask, "Sensor Task", 4096, NULL, 1, NULL);
  xTaskCreate(BuzzerTask, "Buzzer Task", 2048, NULL, 1, NULL);
}

void loop() {
  // Handle BLE disconnection/reconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = false;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = true;
  }
  
  vTaskDelay(100 / portTICK_PERIOD_MS);
}