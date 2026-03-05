#include <Servo.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>

// ===================== PINS (YOUR ROBOT) =====================
const int SERVO_PIN    = 3;   // steering servo
const int SOLENOID_PIN = 2;   // MOSFET gate to solenoid valve

// ===================== SERVO SETTINGS =========================
const int SERVO_CENTER = 90;   // straight ahead (your note)
const int SERVO_MIN    = 60;   // clamp (adjust if needed)
const int SERVO_MAX    = 120;

// ===================== CONTROL SETTINGS =======================
// We'll "lock" desired heading at startup to whatever direction robot faces.
// This is the easiest way to pass the pickup/rotate feedback test reliably.
bool lockHeadingOnStart = true;
float desiredHeadingDeg = 0.0;

// Proportional gain: start small, increase until servo reacts clearly when you rotate robot.
float Kp_heading = 0.8;  // try 0.5 to 2.0

// ===================== SOLENOID TIMING ========================
// Tune these for your pneumatics. Longer ON usually = more push.
const unsigned long SOL_ON_MS  = 300;
const unsigned long SOL_OFF_MS = 200;

// ===================== SENSORS (LAB 3 STYLE) ==================
LIS3MDL mag;
LSM6 imu;

// From your Lab 3 code (may not be perfect for your exact sensor, but works as a start)
LIS3MDL::vector<int16_t> m_min = { -4960,  +1959,  -5915};
LIS3MDL::vector<int16_t> m_max = { -2644,  +4165,  -3989};

Servo steer;

// ===================== STATE ==========================
bool solenoidOn = false;
unsigned long lastSolToggleMs = 0;

unsigned long lastPrintMs = 0;
const unsigned long PRINT_MS = 120;

// ----------------- helper: wrap error to [-180,180] -----------
float wrapDeg180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

// ----------------- heading function (from Lab 3) --------------
template <typename T> float computeHeading(LIS3MDL::vector<T> from)
{
  LIS3MDL::vector<int32_t> temp_m = {mag.m.x, mag.m.y, mag.m.z};

  // copy acceleration readings
  LIS3MDL::vector<int16_t> a = {imu.a.x, imu.a.y, imu.a.z};

  // subtract offset (average of min and max)
  temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
  temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
  temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

  // compute E and N
  LIS3MDL::vector<float> E;
  LIS3MDL::vector<float> N;
  LIS3MDL::vector_cross(&temp_m, &a, &E);
  LIS3MDL::vector_normalize(&E);
  LIS3MDL::vector_cross(&a, &E, &N);
  LIS3MDL::vector_normalize(&N);

  // compute heading
  float heading = atan2(LIS3MDL::vector_dot(&E, &from),
                        LIS3MDL::vector_dot(&N, &from)) * 180 / PI;
  if (heading < 0) heading += 360;
  return heading;
}

float getHeadingDeg() {
  return computeHeading((LIS3MDL::vector<int>){1, 0, 0});
}

// ----------------- solenoid pulsing ---------------------------
void updateSolenoidPulse() {
  unsigned long now = millis();
  unsigned long interval = solenoidOn ? SOL_ON_MS : SOL_OFF_MS;

  if (now - lastSolToggleMs >= interval) {
    lastSolToggleMs = now;
    solenoidOn = !solenoidOn;
    digitalWrite(SOLENOID_PIN, solenoidOn ? HIGH : LOW);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Actuators
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

  steer.attach(SERVO_PIN);
  steer.write(SERVO_CENTER);

  // Sensors
  if (!mag.init()) {
    Serial.println("ERROR: LIS3MDL magnetometer not detected. Check wiring.");
    while (1) {}
  }
  mag.enableDefault();

  if (!imu.init()) {
    Serial.println("ERROR: LSM6 IMU not detected. Check wiring.");
    while (1) {}
  }
  imu.enableDefault();

  // Read heading once on start to set the desired direction
  mag.read();
  imu.read();
  float h0 = getHeadingDeg();
  if (lockHeadingOnStart) desiredHeadingDeg = h0;

  Serial.println("Verif2 HeadingHold + ForwardPulse starting...");
  Serial.print("Desired heading = ");
  Serial.println(desiredHeadingDeg, 1);

  // quick solenoid kick so you can hear it
  digitalWrite(SOLENOID_PIN, HIGH);
  delay(200);
  digitalWrite(SOLENOID_PIN, LOW);
  solenoidOn = false;
  lastSolToggleMs = millis();
}

void loop() {
  // ---- Read sensors ----
  mag.read();
  imu.read();
  float heading = getHeadingDeg();

  // ---- Feedback steering (THIS is the Verif2 pickup/rotate behavior) ----
  float error = wrapDeg180(desiredHeadingDeg - heading);
  int servoCmd = (int)(SERVO_CENTER + Kp_heading * error);

  if (servoCmd < SERVO_MIN) servoCmd = SERVO_MIN;
  if (servoCmd > SERVO_MAX) servoCmd = SERVO_MAX;

  steer.write(servoCmd);

  // ---- Forward motion attempt (open-loop solenoid pulsing) ----
  updateSolenoidPulse();

  // ---- Serial print ----
  if (millis() - lastPrintMs >= PRINT_MS) {
    lastPrintMs = millis();
    Serial.print("heading=");
    Serial.print(heading, 1);
    Serial.print("  err=");
    Serial.print(error, 1);
    Serial.print("  servo=");
    Serial.print(servoCmd);
    Serial.print("  solenoid=");
    Serial.println(solenoidOn ? "ON" : "OFF");
  }
}










