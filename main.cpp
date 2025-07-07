
#include <ECE3.h>
#include <cmath>

#define LED 75

// Sensor calibration values
const int sensorMin[8] = {596, 527, 573, 596, 550, 573, 575, 667};
const int sensorMax[8] = {1802, 1444, 1397, 1113, 1065, 1563, 1224, 1833};

// Sensor weights (before and after 180° turn)
const int weightsBefore[8] = {-15, -10, -8, -11, 14, 16, 18, 20};
const int weightsAfter[8]  = {-20, -18, -16, -14, 11, 8, 10, 15};
int weights[8];

// Motor control pins
const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;
const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// PID control constants
const float kp = 0.0355;
const float kd = 0.1775;

// Phantom crosspiece logic
bool wasAllHigh = false;
bool phantomStreakActive = false;
int phantomEventCount = 0;
bool hasTurned = false;

// Sensor data
uint16_t sensorValues[8];
int lastError = 0;

// Perform a 180° turn in place
void do180Turn() {
  const int turnSpd = 100;
  digitalWrite(left_dir_pin,  LOW);
  digitalWrite(right_dir_pin, HIGH);
  analogWrite(left_pwm_pin,  turnSpd);
  analogWrite(right_pwm_pin, turnSpd);
  delay(600);
  analogWrite(left_pwm_pin,  0);
  analogWrite(right_pwm_pin, 0);
}

// Stop the car and indicate with LED
void stopAndHalt() {
  digitalWrite(LED, HIGH);
  analogWrite(left_pwm_pin,  0);
  analogWrite(right_pwm_pin, 0);
  while (true); // infinite loop
}

// Initial setup
void setup() {
  ECE3_Init();
  Serial.begin(9600);
  delay(2000);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(left_nslp_pin,  OUTPUT);
  pinMode(left_dir_pin,   OUTPUT);
  pinMode(left_pwm_pin,   OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin,  OUTPUT);
  pinMode(right_pwm_pin,  OUTPUT);

  digitalWrite(left_nslp_pin,  HIGH);
  digitalWrite(right_nslp_pin, HIGH);
  digitalWrite(left_dir_pin,   LOW);
  digitalWrite(right_dir_pin,  LOW);

  // Initialize weights for outbound trip
  for (int i = 0; i < 8; i++) {
    weights[i] = weightsBefore[i];
  }
}

// Main control loop
void loop() {
  const int baseSpd = 25;
  ECE3_read_IR(sensorValues);

  int error = 0;
  int highCount = 0;

  for (int i = 0; i < 8; i++) {
    int adjusted = sensorValues[i] - sensorMin[i];
    if (adjusted > 0) {
      int norm = (adjusted * 1000) / sensorMax[i];
      if (norm > 880) highCount++;
      error += norm * weights[i];
    }
  }

  error = error / 8;
  bool nowAllHigh = (highCount >= 6);

  // Phantom crosspiece detection
  if (nowAllHigh) {
    if (wasAllHigh && !phantomStreakActive) {
      phantomStreakActive = true;
      phantomEventCount++;
      lastError = error;

      if (phantomEventCount == 1) {
        do180Turn();
        hasTurned = true;
        for (int i = 0; i < 8; i++) {
          weights[i] = weightsAfter[i];
        }
        return;
      } else if (phantomEventCount == 2) {
        stopAndHalt();
      }
    }
  } else {
    phantomStreakActive = false;
  }

  wasAllHigh = nowAllHigh;

  // PID calculations
  int dError = error - lastError;
  lastError = error;
  float steering_corr = kp * error + kd * dError;

  int leftSpd  = baseSpd - steering_corr;
  int rightSpd = baseSpd + steering_corr;

  // Direction control
  if (leftSpd < 0) {
    leftSpd = 1.5 * abs(leftSpd);
    digitalWrite(left_dir_pin, HIGH);
  } else {
    digitalWrite(left_dir_pin, LOW);
  }

  if (rightSpd < 0) {
    rightSpd = 1.5 * abs(rightSpd);
    digitalWrite(right_dir_pin, HIGH);
  } else {
    digitalWrite(right_dir_pin, LOW);
  }

  analogWrite(left_pwm_pin,  leftSpd);
  analogWrite(right_pwm_pin, rightSpd);
}
