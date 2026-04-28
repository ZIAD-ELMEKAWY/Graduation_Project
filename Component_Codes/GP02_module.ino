/*
 * ESP32C3 GP02 Monitor 
 * @auther : Ziad-Elmekawy
 * @file   : Hardware/GPS
 * @date   : 18, November, 2025
 * @proj   : AURA Health Monitor
 * @desc   : Get from GP02 location, Speed, Date, and Time
 */

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// ------------ GPS Setup ------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// Adjust pins for ESP32-C3
#define GPS_RX 20  // ESP32-C3 RX <-- GP-02 TX
#define GPS_TX 21  // ESP32-C3 TX --> GP-02 RX

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("ESP32-C3 GP-02 GPS Data Reader");
  Serial.println("--------------------------------------");
  Serial.println("Waiting for GPS fix...");
}

void loop() {
  // Read and decode GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // If no GPS data received
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("GPS not detected! Check wiring and power.");
    delay(2000);
    return;
  }

  // Print data when GPS info is valid
  Serial.print("Latitude:  ");
  Serial.println(gps.location.lat(), 6);

  Serial.print("Longitude: ");
  Serial.println(gps.location.lng(), 6);
  Serial.print("Altitude:  ");
  Serial.print(gps.altitude.meters());
  Serial.println(" m");

  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.print("Date: ");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.println(gps.date.year());

    Serial.print("Time (Cairo): ");
    Serial.print(gps.time.hour() + 3);
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
  }

  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());

  Serial.print("Speed: ");
  Serial.print(gps.speed.kmph());
  Serial.println(" km/h");

  Serial.println("--------------------------------------");
  delay(1000);
}
