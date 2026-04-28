/*
 * ESP32C3 MPU6050 Monitor 
 * @auther : Ziad-Elmekawy
 * @file   : Hardware/Accelerometer and Gyroscope 
 * @date   : 18, November, 2025
 * @proj   : AURA Health Monitor
 * @desc   : Get from MPU6050 Accelerometer and Gyroscope 
 */

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Thresholds For motion, still, and shake
float motionThreshold = 0.15;   // g-change to detect motion
float shakeThreshold  = 0.5;    // g-change to detect shake
unsigned long stillDelay = 3000; // ms to confirm still state

// Variables For motion, still, and shake
float prevAx = 0, prevAy = 0, prevAz = 0;
unsigned long lastMotionTime = 0;
// Variables For Step count
int stepCount = 0;
unsigned long lastStepTime = 0;
// Variables For Screen
// n -> Screen ON
// f -> Screen OFF
unsigned char screen_status = 'n' ;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);  // SDA=8, SCL=9 for ESP32-C3 Dev module Super mini
  mpu.initialize();

}

/* ------------- Function Declaration ------------------- */
void Screen_Monitor (void);
void Step_Count (void);
void Detect_Movement (void);

void loop() {
  Screen_Monitor();
  Detect_Movement();
  Step_Count();
}

/* ------------- Function Definition ------------------- */
void Screen_Monitor (void)
{
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to g’s
  float accX = ax / 16384.0;
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;

  // Compute change since last reading for Screen on , off 
  float pitch = atan(accX / sqrt(accY * accY + accZ * accZ)) * 180/PI;
  float roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * 180/PI;
  // Screen ON , OFF
  if((roll >= -50) && (roll <= 15))
  {
    // this condition for Screen ON
    if(screen_status == 'f')
    {
      Serial.println("Screen ON");
      screen_status = 'n' ;
    }
    else
    {
      // Nothing
    }
  }
  else if( roll > 15 )
  {
    // this condition for Screen OFF
    if(screen_status == 'n')
    {
      Serial.println("Screen OFF");
      screen_status = 'f' ;
    }
    else
    {
      // Nothing
    } 
  }
  else 
  {
    // this condition for Screen OFF
    if(screen_status == 'n')
    {
      Serial.println("Screen OFF");
      screen_status = 'f' ;
    }
    else
    {
      // Nothing
    } 
  }
  // Store previous readings
  prevAx = accX;
  prevAy = accY;
  prevAz = accZ;
}

void Step_Count (void)
{
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to g’s
  float accX = ax / 16384.0;
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;
  // Compute change since last reading for Step Count 
  float diff = accZ - prevAz;
  unsigned long now = millis();
  // Detect Step
  if (diff > 0.25 && (now - lastStepTime) > 300) {
    stepCount++;
    lastStepTime = now;
    Serial.print("Step detected! Total = ");
    Serial.println(stepCount);
  }
  // Store previous readings
  prevAx = accX;
  prevAy = accY;
  prevAz = accZ;
}

void Detect_Movement (void)
{
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to g’s
  float accX = ax / 16384.0;
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;
  // Compute change since last reading For motion, still, and shake
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
}
