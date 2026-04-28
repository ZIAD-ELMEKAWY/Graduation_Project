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
  // Convert to 'g' and '°/s'
  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;
  float gyrY = gy / 131.0;

  // Calculate pitch angle
  float pitch = atan(axg / sqrt(ayg * ayg + azg * azg)) * 180 / PI;
  float prevPitch = 0;

  Serial.print("Accel (g): ");
  Serial.print(ax/16384.0); Serial.print(", ");
  Serial.print(ay/16384.0); Serial.print(", ");
  Serial.print(az/16384.0);

  Serial.print(" | Gyro (°/s): ");
  Serial.print(gx/131.0); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.print(gz/131.0);Serial.print(", ");
  Serial.print(gyrY);

  Serial.print(" | Pitch: ");
  Serial.print(pitch);Serial.println(", ");


  // Detect standing/sitting
  if ((prevPitch > 50) && (pitch < 20) || (gyrY < -60)) {
    Serial.println("🧍 Standing up detected!");
  }
  else if ((prevPitch < 20) && (pitch > 50) || (gyrY > 60)) {
    Serial.println("💺 Sitting down detected!");
  }
  else 
  {
    Serial.println("Nothing!");
  }

  prevPitch = pitch;


  delay(100);
}
