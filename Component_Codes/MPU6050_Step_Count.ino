#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
int stepCount = 0;
float prevAccZ = 0;
unsigned long lastStepTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(8,9);
  mpu.initialize();
  Serial.println("Step Counter Ready");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accZ = az / 16384.0;
  float diff = accZ - prevAccZ;
  unsigned long now = millis();

  if (diff > 0.25 && (now - lastStepTime) > 300) {
    stepCount++;
    lastStepTime = now;
    Serial.print("Step detected! Total = ");
    Serial.println(stepCount);
  }

  prevAccZ = accZ;
  delay(50);
}
