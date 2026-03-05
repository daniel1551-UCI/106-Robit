#include <Servo.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>

const int SERVO_PIN = 3;
const int SERVO_CENTER = 90;
const int SERVO_MIN = 60;
const int SERVO_MAX = 120;

float Kp_heading = 0.8;

LIS3MDL mag;
LSM6 imu;

// Use your Lab 3 calibration values as a starting point
LIS3MDL::vector<int16_t> m_min = { -4960,  +1959,  -5915};
LIS3MDL::vector<int16_t> m_max = { -2644,  +4165,  -3989};

Servo steer;
float desiredHeadingDeg = 0;

float wrapDeg180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

// ===== Lab 3 heading function =====
template <typename T> float computeHeading(LIS3MDL::vector<T> from)
{
  LIS3MDL::vector<int32_t> temp_m = {mag.m.x, mag.m.y, mag.m.z};
  LIS3MDL::vector<int16_t> a = {imu.a.x, imu.a.y, imu.a.z};

  temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
  temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
  temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

  LIS3MDL::vector<float> E, N;
  LIS3MDL::vector_cross(&temp_m, &a, &E);
  LIS3MDL::vector_normalize(&E);
  LIS3MDL::vector_cross(&a, &E, &N);
  LIS3MDL::vector_normalize(&N);

  float heading = atan2(LIS3MDL::vector_dot(&E, &from),
                        LIS3MDL::vector_dot(&N, &from)) * 180 / PI;
  if (heading < 0) heading += 360;
  return heading;
}

float getHeadingDeg() {
  return computeHeading((LIS3MDL::vector<int>){1, 0, 0});
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  steer.attach(SERVO_PIN);
  steer.write(SERVO_CENTER);

  if (!mag.init()) { Serial.println("ERROR: LIS3MDL not detected"); while(1){} }
  mag.enableDefault();
  if (!imu.init()) { Serial.println("ERROR: LSM6 not detected"); while(1){} }
  imu.enableDefault();

  mag.read(); imu.read();
  desiredHeadingDeg = getHeadingDeg(); // lock heading at start

  Serial.print("Desired heading locked at: ");
  Serial.println(desiredHeadingDeg, 1);
}

void loop() {
  mag.read(); imu.read();
  float heading = getHeadingDeg();
  float err = wrapDeg180(desiredHeadingDeg - heading);

  int servoCmd = (int)(SERVO_CENTER + Kp_heading * err);
  if (servoCmd < SERVO_MIN) servoCmd = SERVO_MIN;
  if (servoCmd > SERVO_MAX) servoCmd = SERVO_MAX;

  steer.write(servoCmd);

  Serial.print("heading=");
  Serial.print(heading, 1);
  Serial.print(" err=");
  Serial.print(err, 1);
  Serial.print(" servo=");
  Serial.println(servoCmd);

  delay(50);
}


