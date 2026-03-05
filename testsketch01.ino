#include <Servo.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>

// This code passed the steering test!!

// ===================== YOUR PINS =====================
const int SERVO_PIN    = 3;   // steering servo
const int SOLENOID_PIN = 2;   // MOSFET gate for solenoid/piston valve

// ===================== SERVO SETTINGS =================
const int SERVO_CENTER = 90;  // straight ahead (your note)
const int SERVO_LEFT   = 60;
const int SERVO_RIGHT  = 120;

// ===================== SOLENOID TIMING =================
// Start with longer pulses so you actually see motion
const unsigned long SOL_ON_MS  = 300;
const unsigned long SOL_OFF_MS = 200;

// ===================== MAG/IMU (OPTIONAL PRINT) =========
// Using your Lab 3 style setup
LIS3MDL mag;
LSM6 imu;

// If these calibration values aren’t correct for YOUR sensor,
// heading may be off — but that's okay for an actuation test.
LIS3MDL::vector<int16_t> m_min = { -4960,  +1959,  -5915};
LIS3MDL::vector<int16_t> m_max = { -2644,  +4165,  -3989};

Servo steer;

// ===================== STATE ===========================
bool solenoidOn = false;
unsigned long lastSolToggleMs = 0;

unsigned long lastPrintMs = 0;
const unsigned long PRINT_MS = 150;

// ---------- Heading functions (from your Lab 3 code) ----------
template <typename T> float computeHeading(LIS3MDL::vector<T> from)
{
  LIS3MDL::vector<int32_t> temp_m = {mag.m.x, mag.m.y, mag.m.z};
  LIS3MDL::vector<int16_t> a = {imu.a.x, imu.a.y, imu.a.z};

  temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
  temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
  temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

  LIS3MDL::vector<float> E;
  LIS3MDL::vector<float> N;
  LIS3MDL::vector_cross(&temp_m, &a, &E);
  LIS3MDL::vector_normalize(&E);
  LIS3MDL::vector_cross(&a, &E, &N);
  LIS3MDL::vector_normalize(&N);

  float heading = atan2(LIS3MDL::vector_dot(&E, &from),
                        LIS3MDL::vector_dot(&N, &from)) * 180 / PI;
  if (heading < 0) heading += 360;
  return heading;
}

float getHeadingDeg()
{
  return computeHeading((LIS3MDL::vector<int>){1, 0, 0});
}

// ---------- Solenoid pulsing ----------
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

  // Solenoid output
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

  // Servo
  steer.attach(SERVO_PIN);
  steer.write(SERVO_CENTER);

  // Magnetometer + IMU init (optional but nice)
  if (!mag.init()) {
    Serial.println("ERROR: LIS3MDL magnetometer not detected (check wiring).");
    // Still allow solenoid/servo to run even if compass fails:
  } else {
    mag.enableDefault();
  }

  if (!imu.init()) {
    Serial.println("ERROR: LSM6 IMU not detected (check wiring).");
  } else {
    imu.enableDefault();
  }

  // Quick steering wiggle so you SEE it works
  Serial.println("Actuation Test Starting...");
  Serial.println("Wiggling steering, then pulsing solenoid...");
  steer.write(SERVO_LEFT);  delay(500);
  steer.write(SERVO_RIGHT); delay(500);
  steer.write(SERVO_CENTER);delay(300);

  // Quick solenoid kick so you HEAR it immediately
  digitalWrite(SOLENOID_PIN, HIGH);
  delay(200);
  digitalWrite(SOLENOID_PIN, LOW);
  solenoidOn = false;
  lastSolToggleMs = millis();
}

void loop() {
  // Keep steering centered for straight-ish travel
  steer.write(SERVO_CENTER);

  // Pulse solenoid continuously (NO limit switches)
  updateSolenoidPulse();

  // Read + print heading if sensors are alive
  float heading = -1.0;
  if (mag.init() && imu.init()) { // lightweight check; safe enough for this test
    mag.read();
    imu.read();
    heading = getHeadingDeg();
  }

  if (millis() - lastPrintMs >= PRINT_MS) {
    lastPrintMs = millis();
    Serial.print("solenoid=");
    Serial.print(solenoidOn ? "ON" : "OFF");
    Serial.print("  servo=");
    Serial.print(SERVO_CENTER);
    Serial.print("  heading=");
    Serial.println(heading, 1);
  }
}