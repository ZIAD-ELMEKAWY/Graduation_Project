#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Thresholds
float motionThreshold = 0.15;   // g-change to detect motion
float shakeThreshold  = 0.5;    // g-change to detect shake
unsigned long stillDelay = 3000; // ms to confirm still state

// Variables
float prevAx = 0, prevAy = 0, prevAz = 0;
unsigned long lastMotionTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);  // SDA=8, SCL=9 for ESP32-C3 Dev module Super mini
  mpu.initialize();

  if (mpu.testConnection())
    Serial.println("✅ MPU6050 connected!");
  else
    Serial.println("❌ Connection failed!");

  delay(1000);
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to g’s
  float accX = ax / 16384.0;
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;

  // Compute change since last reading
  float dX = abs(accX - prevAx);
  float dY = abs(accY - prevAy);
  float dZ = abs(accZ - prevAz);
  float delta = max(dX, max(dY, dZ));

  // Detect motion
  if (delta > motionThreshold && delta < shakeThreshold) {
    Serial.println("🏃 Motion detected!");
    lastMotionTime = millis();
  }
  // Detect shake or sudden movement
  else if (delta >= shakeThreshold) {
    Serial.println("⚠️  Shake or impact detected!");
    lastMotionTime = millis();
  }
  // Detect stillness
  else if (millis() - lastMotionTime > stillDelay) {
    Serial.println("🧘 Device is still.");
    lastMotionTime = millis(); // reset timer to avoid repeat prints
  }

  // Store previous readings
  prevAx = accX;
  prevAy = accY;
  prevAz = accZ;

  delay(100);
}
