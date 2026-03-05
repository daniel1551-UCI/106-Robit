#include <Servo.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>

// --- FORWARD DECLARATIONS ---
// These lines tell the Arduino that these functions exist, fixing the "no matching function" error.
template <typename T> float computeHeading(LIS3MDL::vector<T> from);
float computeHeading();
// ----------------------------

LIS3MDL mag;
LSM6 imu;

LIS3MDL::vector<int16_t> m_min = { +1282,  +1664,   -815};
LIS3MDL::vector<int16_t> m_max = { +2568,  +2881,   -101};

Servo myservo;

int servoPin = 3;       // Pin that the servomotor is connected to
int solenoidPin = 2;    // Pin that the mosfet is conected to
int switchPin = 4;      // Pin that the switch is conected to
int pos = 0;            // variable to store the servo position
int switchState;        // variable that stores the Reed switch state
int solenoidState = LOW;// variable that stores if solenoid is on or off         
unsigned long previousMillis = 0; // will store last time solenoid was updated
const long interval = 1000;       // interval at which to turn solenoid on and off (milliseconds)


void setup() {
  myservo.attach(servoPin);       // attaches the servo on pin 9 to the servo object
  pinMode(solenoidPin, OUTPUT);   // Sets the pin as an output
  pinMode(switchPin, INPUT_PULLUP); // Sets the pin as an input_pullup
  Serial.begin(9600);             // starts serial communication @ 9600 bps
  Wire.begin();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize LIS3MDL magnetometer!");
    while (1);
  }
  mag.enableDefault();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize LSM6 IMU!");
    while (1);
  }
  imu.enableDefault();
}


void loop() {

  ////////////// MAGNETOMETER ///////////////////////////////////////////////////

  mag.read();
  imu.read();
  
  // This call works now because of the forward declaration at the top
  float heading = computeHeading();
  
  ////////////// ROBOTIC VOR (CALIBRATED) ///////////////////////////////////////
  
  // 1. Adjust Heading for "Midrange" usage
  // The compass is 0-360, but the servo is limited. 
  // We want 0 degrees (North) to be the center of our servo range.
  // We convert headings like 350 into -10 so they are continuous across North.
  float adjustedHeading = heading;
  if (adjustedHeading > 180) {
    adjustedHeading -= 360; 
  }
  
  // 2. Calculate "Equal and Opposite" Physical Angle
  // We center the servo at 90 degrees (Midrange).
  // If heading turns positive (Right), Servo must turn negative (Left).
  // Target Actual Angle = 90 - Heading
  float targetActualAngle = 90.0 - adjustedHeading;

  // 3. Apply Calibration Equation (y = 0.7x)
  // x = targetActualAngle
  // y = servoCommand
  float servoCommand = 0.7 * targetActualAngle;

  // 4. Safety Constraints
  // Ensure we don't send commands the servo cannot handle
  if (servoCommand > 180) servoCommand = 180;
  if (servoCommand < 0) servoCommand = 0;

  // Actuate Servo
  myservo.write((int)servoCommand); 
  delay(10); 


  ////////////// SOLENOID VALVE ///////////////////////////////////////////////////
  unsigned long currentMillis = millis(); 
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (solenoidState == LOW) {
      solenoidState = HIGH;
    } else {
      solenoidState = LOW;
    }
    digitalWrite(solenoidPin, solenoidState); //Switch Solenoid ON/oFF
  }

  ////////////// REED SWITCH ///////////////////////////////////////////////////
  switchState = digitalRead(switchPin);
  
  ////////////// Serial Print (CONTINUOUS) //////////////////////////////////////
  // Print 3 variables in columns continuously
  
  Serial.print(currentMillis);
  Serial.print("\t");         // Tab creates the column spacing
  Serial.print(solenoidState);
  Serial.print("\t");
  Serial.println(heading);    // Newline at the end of the row
}


// --- FUNCTION DEFINITIONS ---

// Helper function to call the template with a default vector
float computeHeading()
{
  // We create the vector explicitly to avoid syntax errors with the casting
  LIS3MDL::vector<int> defaultVector = {1, 0, 0};
  return computeHeading(defaultVector);
}

// Main template function for magnetometer
template <typename T> float computeHeading(LIS3MDL::vector<T> from)
{
  LIS3MDL::vector<int32_t> temp_m = {mag.m.x, mag.m.y, mag.m.z};
  // copy acceleration readings from LSM6::vector into an LIS3MDL::vector
  LIS3MDL::vector<int16_t> a = {imu.a.x, imu.a.y, imu.a.z};
  // subtract offset (average of min and max) from magnetometer readings
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
  float heading = atan2(LIS3MDL::vector_dot(&E, &from), LIS3MDL::vector_dot(&N, &from)) * 180 / PI;
  if (heading < 0) heading += 360;
  return heading;
}