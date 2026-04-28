/*
 * ESP32C3 Battery Monitor with ASCII Icon 
 * @auther : Ziad-Elmekawy
 * @file   : Hardware/Battery Percentage
 * @date   : 18, November, 2025
 * @proj   : AURA Health Monitor
 * @desc   : Measure the battery percentage 
              and display value on the screen
 */

// ADC pin  = ADC1 in ESP32-C3
#define ANALOG_IN_PIN 1      

// REF_VOLTAGE according to the highest value which ESP32-C3 can measure
#define REF_VOLTAGE    3.3
// ESP32-C3 ADC bit resolution = 12-bit
#define ADC_RESOLUTION 4095.0
// Resistor values in voltage divider circuit 
#define R1 30000.0
#define R2 7500.0
// Max and Min voltage value to present the battery percentage
#define BATTERY_MAX_VOLTAGE 3.5
#define BATTERY_MIN_VOLTAGE 2.0

void setup() {
  Serial.begin(115200);
}

void loop() {

  int adc_value = analogRead(ANALOG_IN_PIN);
  float voltage_adc = (adc_value * REF_VOLTAGE) / ADC_RESOLUTION;
  float battery_voltage = voltage_adc * (R1 + R2) / R2;

  float percentage = (battery_voltage - BATTERY_MIN_VOLTAGE) * 100.0 /
                     (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);

  if (percentage > 100) percentage = 100;
  if (percentage < 0) percentage = 0;

  // Battery level zone
  int level;
  if (percentage >= 100)      level = 5;
  else if (percentage >= 75)  level = 4;
  else if (percentage >= 50)  level = 3;
  else if (percentage >= 25)  level = 2;
  else if (percentage >= 10)  level = 1;
  else                        level = 0;

  // ASCII battery icons
  String icons[6] = {
    "[|     ] 0%",      // level 0
    "[||    ] 10%",     // level 1
    "[|||   ] 25%",     // level 2
    "[||||  ] 50%",     // level 3
    "[||||| ] 75%",     // level 4
    "[||||||] 100%"     // level 5
  };

  // Print results
  Serial.print("Battery Voltage: ");
  Serial.print(battery_voltage, 2);
  Serial.print(" V   |   ");
  
  Serial.print("Percentage: ");
  Serial.print(percentage, 1);
  Serial.print("%   |   ");

  Serial.print("Icon: ");
  Serial.println(icons[level]);

  delay(500);
}

