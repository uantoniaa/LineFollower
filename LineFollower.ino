#include <QTRSensors.h>

// Define the motor driver pins
const int m11Pin = 7;    // Motor 1 input pin 1
const int m12Pin = 6;    // Motor 1 input pin 2
const int m21Pin = 5;    // Motor 2 input pin 1
const int m22Pin = 4;    // Motor 2 input pin 2
const int m1Enable = 11; // Motor 1 speed control pin (PWM)
const int m2Enable = 10; // Motor 2 speed control pin (PWM)

// Initialize motor speed variables
int m1Speed = 0;
int m2Speed = 0;

// PID control parameters
float kp = 35;   // Proportional gain
float ki = 0.05; // Integral gain
float kd = 145;  // Derivative gain

// PID control variables
int p = 0;           // Proportional term
int i = 0;           // Integral term
int d = 0;           // Derivative term
int error = 0;       // Error term
int lastError = 0;   // Previous error term for derivative calculation

// Motor speed limits
const int maxSpeed = 255;  // Maximum motor speed
const int minSpeed = -255; // Minimum motor speed (for reverse)
const int baseSpeed = 255; // Base speed for motors

// Initialize QTR sensor object
QTRSensors qtr;

// Define sensor array parameters
const int sensorCount = 6;              // Number of sensors in the array
int sensorValues[sensorCount];          // Array to store sensor readings
int sensors[sensorCount] = {0, 0, 0, 0, 0, 0};

// Calibration and line detection parameters
const int speedForCalibration = 150;                // Motor speed during calibration
const int thresholdForBlackLineDetection = 600;     // Sensor threshold for detecting black line
const int thresholdForNonBlackLineDetection = 450;  // Sensor threshold for detecting non-black line
const int lowestSensorReading = 0;                  // Lowest possible sensor reading
const int highestSensorReading = 5000;              // Highest possible sensor reading

// PID error range
const int minErrorLimit = -50;
const int maxErrorLimit = 50;

// Number of calibration cycles
const int numOfCalibrationCycles = 255;
int motorSpeed;

void setup() {
  // Initialize motor driver pins as outputs
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);

  // Configure the QTR sensors
  qtr.setTypeAnalog(); // Set sensor type to analog
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);

  delay(500); // Short delay before starting calibration

  // Start calibration process
  pinMode(LED_BUILTIN, OUTPUT); // Setup built-in LED for indicating calibration status
  digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate calibration is in progress
  setMotorSpeed(speedForCalibration, -speedForCalibration); // Set motor speeds for calibration

  // Calibration loop
  for (uint16_t i = 0; i < numOfCalibrationCycles; ++i) {
    qtr.calibrate(); // Perform sensor calibration

    qtr.read(sensorValues); // Read sensor values
    // Logic to turn the robot based on sensor readings
    bool isFirstSensorOnBlack = sensorValues[0] > thresholdForBlackLineDetection;
    bool isSecondSensorOffBlack = sensorValues[1] < thresholdForNonBlackLineDetection;
    bool isLastSensorOnBlack = sensorValues[sensorCount - 1] > thresholdForBlackLineDetection;
    bool isPenultimateSensorOffBlack = sensorValues[sensorCount - 2] < thresholdForNonBlackLineDetection;
    if (isFirstSensorOnBlack && isSecondSensorOffBlack) {
        setMotorSpeed(-speedForCalibration, speedForCalibration);
    }
    if (isLastSensorOnBlack && isPenultimateSensorOffBlack) {
        setMotorSpeed(speedForCalibration, -speedForCalibration);
    }
  }

  digitalWrite(LED_BUILTIN, LOW); // Turn off LED to indicate calibration is complete
}

void loop() {
  pidControl(kp, ki, kd); // Call the PID control function
  setMotorSpeed(m1Speed, m2Speed); // Set motor speeds based on PID output
}

void pidControl(float kp, float ki, float kd) {
  // Read the line position and map it to an error value
  error = map(qtr.readLineBlack(sensorValues), lowestSensorReading, highestSensorReading, minErrorLimit, maxErrorLimit);

  // Calculate PID terms
  p = error;                 // Proportional term is simply the error
  i = i + error;             // Integral term is the cumulative error
  d = error - lastError;     // Derivative term is the change in error
  lastError = error;         // Update last error for next derivative calculation

  // Calculate motor speed adjustment from PID terms
  motorSpeed = kp * p + ki * i + kd * d;

  // Set base motor speeds
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  // Adjust motor speeds based on error direction
  if (error < 0) {
    m1Speed += motorSpeed; // Turn left if error is negative
  }
  else if (error > 0) {
    m2Speed -= motorSpeed; // Turn right if error is positive
  }

  // Constrain motor speeds to maximum and minimum values
  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);
}

void setMotorSpeed(int m1Speed, int m2Speed) {
  // Invert motor speeds for correct direction control
  m1Speed = -m1Speed;
  m2Speed = -m2Speed;

  // Control motor 1
  if (m1Speed == 0) {
    // Stop motor 1 if speed is zero
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, m1Speed);
  }
  else {
    if (m1Speed > 0) {
      // Run motor 1 forward
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, m1Speed);
    }
    if (m1Speed < 0) {
      // Run motor 1 backward
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -m1Speed);
    }
  }

  // Control motor 2
  if (m2Speed == 0) {
    // Stop motor 2 if speed is zero
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, m2Speed);
  }
  else {
    if (m2Speed > 0) {
      // Run motor 2 forward
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, m2Speed);
    }
    if (m2Speed < 0) {
      // Run motor 2 backward
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -m2Speed);
    }
  }
}
