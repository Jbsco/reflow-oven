// Reflow Oven PID Control - Refactored and Simplified with Cooling Servo
#include <PID_v1.h>
#include <Adafruit_MAX31856.h>
#include <ESP32Servo.h>

#define DRDY_PIN  15
#define OUT_PIN   10
#define SERVO_PIN 5
#define FAN_PIN   7
// Button pins
#define PIN_BUTTON_1 11

Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(43,44,18,17);
Servo doorServo;

// Published values for SG90 servos; adjust if needed
int minUs = 550;
int maxUs = 2350;

// PID control variables
double currentTemp, targetTemp, heaterPower, elapsed, loopTime, dT = 0;
PID ovenPID(&currentTemp, &heaterPower, &targetTemp, 20, 0, 1, DIRECT);

// Cooling control variables
double coolingRate = -5.0; // °C/s
const double coolingTarget = 30.0; // Target temp to stop cooling
bool coolingActive = false;
unsigned long coolingStartTime;
double coolingStartTemp;

// Servo PID variables
double expectedTemp, servoOutput;
PID coolingPID(&currentTemp, &servoOutput, &expectedTemp, 2, 0.1, 1.5, REVERSE);

// Reflow profile segments: {time in sec, temp in C}
const double profile[][2] = {
  //{0,50}, {800, 220}
  {0,50},{400,150},{800,150}
};
const int profileCount = sizeof(profile) / sizeof(profile[0]);

// Timers
unsigned long startTime;
bool running = false, startFlag = false;

void IRAM_ATTR buttonISR1(){
    if (not startFlag){
        startFlag = 1;
    }
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
    analogWrite(OUT_PIN, 0); // turn off heating
    Serial.println("Cooling started");
}

void manageCooling() {
    double timeSinceCooling = (millis() - coolingStartTime) / 1000.0;
    expectedTemp = coolingStartTemp + coolingRate * timeSinceCooling;
    if (expectedTemp < 0) expectedTemp = 0;
    coolingPID.Compute();
    doorServo.write((int)servoOutput);
    if (currentTemp <= coolingTarget) {
        coolingActive = false;
        running = false;
        targetTemp = 0;
        doorServo.write(0); // close door
        delay(25);
        Serial.println("Cooling complete");
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(OUT_PIN, OUTPUT);

    pinMode(PIN_BUTTON_1, INPUT_PULLDOWN);
    // link the button 1 functions.
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_1), buttonISR1, FALLING);

    thermocouple.begin();
    thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);
    thermocouple.setConversionMode(MAX31856_CONTINUOUS);

    ovenPID.SetMode(AUTOMATIC);
    ovenPID.SetOutputLimits(0, 255);  // PWM range
    analogWrite(OUT_PIN, 0);

    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    doorServo.setPeriodHertz(50);      // Standard 50hz servo
    doorServo.attach(SERVO_PIN, minUs, maxUs);
    doorServo.write(0); // Start with door closed
    delay(25);

    coolingPID.SetMode(AUTOMATIC);
    coolingPID.SetOutputLimits(0, 180);  // Servo angle range
}

void loop() {
    unsigned long now = millis();
    dT = now * 0.001 - loopTime;
    loopTime = now * 0.001;

    // Read current temperature
    currentTemp = thermocouple.readThermocoupleTemperature();

    if (!running && startFlag) {
        startTime = millis();
        running = true;
        Serial.println("Reflow started");
    }


    // If cooling phase is active
    if (coolingActive) {
        manageCooling();
    } else if (running){
        elapsed = (millis() - startTime) / 1000.0;
        // Determine the current setpoint from profile
        targetTemp = getTargetTemp(elapsed);

        // Compute PID and apply output
        ovenPID.Compute();
        analogWrite(OUT_PIN, (int)heaterPower);

        // End condition
        if (!coolingActive && elapsed >= profile[profileCount - 1][0]) {
            running = false;
            startFlag = false;
            analogWrite(OUT_PIN, 0);
            Serial.println("Reflow complete");
            startCooling();
        }
    }
    // Optional debug output
    Serial.print("Time: "); Serial.print(loopTime);
    Serial.print(" s, dT: "); Serial.print(dT);
    Serial.print(" s, Temp: "); Serial.print(currentTemp);
    Serial.print(" C, Target: "); Serial.print(targetTemp);
    Serial.print(" C, Output: "); Serial.println(heaterPower);
}