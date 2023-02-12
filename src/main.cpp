#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int mpuInterval = 10;

Adafruit_MPU6050 mpu;

unsigned long currentMillis = 0;

unsigned long previousMpuMillis = 0;

float maxZ = 0;

bool apogeeReached = false;

void setup()
{

  Serial.begin(115200);
  Serial.println("tes");

  pinMode(LED_BUILTIN, OUTPUT);

  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-16G
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

void readMPU()
{
  if (millis() - previousMpuMillis >= mpuInterval)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float accelZ = -a.acceleration.z;

    if (accelZ < 0.5)
    {
      apogeeReached = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }

    if (!apogeeReached)
    {
      if (accelZ > maxZ)
      {
        maxZ = accelZ;
      }
    }

    // Serial.print("0, -50, 50, ");
    Serial.println(maxZ);

    previousMpuMillis = mpuInterval;
  }
}

void loop()
{
  currentMillis = millis();

  readMPU();
}
