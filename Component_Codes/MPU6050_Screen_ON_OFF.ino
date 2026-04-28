#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9); // SDA=8, SCL=9 for ESP32-C3 Dev module Super mini
  Serial.println("Initializing MPU6050...");
  
  mpu.initialize();
  if (mpu.testConnection())
    Serial.println("MPU6050 connected!");
  else
    Serial.println("Connection failed!");
}

void loop() {
  // ax, ay, az --> for acceleration
  // gx, gy, gz --> for Gyrscope
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;

  float pitch = atan(axg / sqrt(ayg * ayg + azg * azg)) * 180/PI;
  float roll  = atan(ayg / sqrt(axg * axg + azg * azg)) * 180/PI;

  Serial.print("Accel (g): ");
  Serial.print(ax/16384.0); Serial.print(", ");
  Serial.print(ay/16384.0); Serial.print(", ");
  Serial.print(az/16384.0);

  Serial.print(" | Gyro (°/s): ");
  Serial.print(gx/131.0); Serial.print(", ");
  Serial.print(gy/131.0); Serial.print(", ");
  Serial.print(gz/131.0);

  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print("°  Roll: ");
  Serial.print(roll);
  Serial.print("°");Serial.print(", ");

  if((roll >= -50) && (roll <= 15))
  {
    Serial.println("Screen ON");
  }
  else if( roll > 15 )
  {
    Serial.println("Screen OFF");
  }
  else 
  {
    Serial.println("Screen OFF");
  }


  delay(500);
}
