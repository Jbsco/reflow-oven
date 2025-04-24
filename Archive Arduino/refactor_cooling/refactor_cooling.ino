// Reflow Oven PID Control - Refactored and Simplified with Cooling Servo
#include <PID_v1.h>
#include <Adafruit_MAX31856.h>
#include <Servo.h>
#include "AVR_PWM.h"

#define DRDY_PIN    5
#define OUT_PIN     10
#define START_PIN   4
#define LOGIC_PIN   2
#define SERVO_PIN   6
#define FAN_PIN   7

Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(9);
Servo doorServo;

AVR_PWM* PWM_Instance;
float frequency = 500;
uint32_t dutyCycle;

bool doneDone = true;

// PID control variables
double currentTemp, targetTemp, heaterPower;
PID ovenPID(&currentTemp, &heaterPower, &targetTemp, 20, 0.1, 1, DIRECT);

// Cooling control variables
double coolingRate = -1.5; // °C/s
const double coolingTarget = 50.0; // Target temp to stop cooling
bool coolingActive = false;
unsigned long coolingStartTime;
double coolingStartTemp;

// Servo PID variables
double expectedTemp, servoOutput;
PID coolingPID(&currentTemp, &servoOutput, &expectedTemp, 5, 0.5, 1, REVERSE);

// Reflow profile segments: {time in sec, temp in C}
const double profile[][2] = {
  //{0,50}, {800, 220}
  {0,50},{400,150},{800,150}
};
const int profileCount = sizeof(profile) / sizeof(profile[0]);

// Timers
unsigned long startTime;
bool running = false;

void setup() {
  Serial.begin(9600);
  PWM_Instance = new AVR_PWM(OUT_PIN, frequency, 0);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(LOGIC_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  PWM_Instance->setPWM_Int(OUT_PIN, frequency, 0);

  thermocouple.begin();
  thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);

  ovenPID.SetMode(AUTOMATIC);
  ovenPID.SetOutputLimits(0, 255);  // PWM range

  doorServo.attach(SERVO_PIN);
  doorServo.write(15); // Start with door closed
  analogWrite(FAN_PIN, 0); // Start with fan off

  coolingPID.SetMode(AUTOMATIC);
  coolingPID.SetOutputLimits(15, 165);  // Servo angle range
}

void loop() {
  if (!running && digitalRead(START_PIN) == LOW) {
    startTime = millis();
    doneDone = false;
    running = true;
    Serial.println("Reflow started");
  }

  if (!running) return;

  // Read current temperature
  currentTemp = thermocouple.readThermocoupleTemperature();
  double elapsed = (millis() - startTime) / 1000.0;

  // If cooling phase is active
  if (coolingActive) {
    manageCooling(elapsed);
  } else {
    // Determine the current setpoint from profile
    targetTemp = getTargetTemp(elapsed);

    // Compute PID and apply output
    ovenPID.Compute();
    PWM_Instance->setPWM_Int(OUT_PIN, frequency, heaterPower/255);
  }

  // Check if cooling should begin
  if (!doneDone && !coolingActive && elapsed >= profile[profileCount - 1][0]) {
    startCooling();
  }
  if (!coolingActive){
  // Optional debug output
  Serial.print("Time: "); Serial.print(elapsed);
  Serial.print(" s, Temp: "); Serial.print(currentTemp);
  Serial.print(" C, Target: "); Serial.print(targetTemp);
  Serial.print(" C, Output: "); Serial.println(heaterPower);
  }

  //delay(100);
}

// Interpolates the target temperature from the profile
double getTargetTemp(double timeSec) {
  for (int i = 1; i < profileCount; i++) {
    double t0 = profile[i - 1][0], t1 = profile[i][0];
    double y0 = profile[i - 1][1], y1 = profile[i][1];
    if (timeSec <= t1) {
      double slope = (y1 - y0) / (t1 - t0);
      return y0 + slope * (timeSec - t0);
    }
  }
  return profile[profileCount - 1][1];
}

void startCooling() {
  coolingActive = true;
  coolingStartTime = millis();
  coolingStartTemp = currentTemp;
  analogWrite(FAN_PIN, 255); // turn on fan
  PWM_Instance->setPWM_Int(OUT_PIN, frequency, 0);
  Serial.println("Cooling started");
}

void manageCooling(double elapsed) {
  double timeSinceCooling = (millis() - coolingStartTime) / 1000.0;
  expectedTemp = coolingStartTemp + coolingRate * timeSinceCooling;
  if(expectedTemp < 0) expectedTemp = 0;
  coolingPID.Compute();
  doorServo.write((int)servoOutput);
  // Optional debug output
  Serial.print("Time: "); Serial.print(elapsed);
  Serial.print(" s, Temp: "); Serial.print(currentTemp);
  Serial.print(" C, Target: "); Serial.print(expectedTemp);
  Serial.print(" C, Output: "); Serial.println((int)servoOutput);

  if (currentTemp <= coolingTarget) {
    coolingActive = false;
    running = false;
    analogWrite(FAN_PIN, 0); // turn off fan
    doorServo.write(15); // close door
    Serial.println("Cooling complete");
    doneDone = true;
  }
}
