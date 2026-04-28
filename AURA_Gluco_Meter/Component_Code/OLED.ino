/* ESP32C3 BLE Monitor 
 * @auther : Ziad-Elmekawy
 * @file   : Hardware/OLED
 * @date   : 24, April, 2026
 * @proj   : AURA Health Monitor
 * @desc   : Measure Temperature body and Blood Glucose
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define BUTTON_PIN 6
#define ONE_WIRE_BUS 3  // DS18B20 data pin → GPIO5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ─────────────────────────────────────────
//  DRAW HELPERS
// ─────────────────────────────────────────

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

// ─────────────────────────────────────────
//  SCREENS
// ─────────────────────────────────────────

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

void showMeasuringScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 24);
  display.println("Measuring");
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

// ─────────────────────────────────────────
//  BUTTON HELPER
// ─────────────────────────────────────────

void waitForButtonPress() {
  while (digitalRead(BUTTON_PIN) == HIGH) delay(10);
  delay(50);
  while (digitalRead(BUTTON_PIN) == LOW);
  delay(50);
}

// ─────────────────────────────────────────
//  READ TEMPERATURE
// ─────────────────────────────────────────

float readTemperature() {
  const int samples = 20;
  const float TEMP_OFFSET = 10.0;  // Adjust after calibration
  float total = 0;
  int validCount = 0;

  sensors.setResolution(12);
  sensors.setWaitForConversion(true);  // Auto-wait 750ms per read

  for (int i = 0; i < samples; i++) {
    sensors.requestTemperatures();
    float t = sensors.getTempCByIndex(0);

    if (t != DEVICE_DISCONNECTED_C && t > 0.0 && t < 50.0) {
      total += t;
      validCount++;
    }
  }

  if (validCount == 0) return DEVICE_DISCONNECTED_C;

  return (total / validCount) + TEMP_OFFSET;
}

// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Wire.begin(8, 9);
  sensors.begin();  // Start DS18B20

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  // Splash screen → 1 second → Press to Start
  showSplashScreen(70, false);
  delay(1000);
  showPressButtonScreen();
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────

void loop() {
  // Wait for button press on "Press to Start" screen
  waitForButtonPress();

  // Show measuring animation
  showMeasuringScreen();
  delay(500);  // Brief pause before reading

  // Read DS18B20
  float temperature = readTemperature();
  Serial.print("Temperature: ");
  Serial.println(temperature);

  // Show measuring dots while waiting for sensor
  // DS18B20 needs ~750ms for 12-bit resolution
  delay(750);

  // Show Blood Glucose (replace 98.5 with real sensor value)
  showBloodGlucoseScreen(98.5);
  waitForButtonPress();

  // Show Temperature from DS18B20
  showTemperatureScreen(temperature);
  waitForButtonPress();

  // Back to Press to Start
  showPressButtonScreen();
}
