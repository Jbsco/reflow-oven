// Reflow Oven PID Control - Refactored and Simplified
#include <PID_v1.h>
#include <Adafruit_MAX31856.h>

#define DRDY_PIN    5
#define OUT_PIN     10
#define START_PIN   4
#define LOGIC_PIN   2

Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(9);

// PID control variables
double currentTemp, targetTemp, heaterPower;
PID ovenPID(&currentTemp, &heaterPower, &targetTemp, 20, 0, 1, DIRECT);

// Reflow profile segments: {time in sec, temp in C}
const double profile[][2] = {
  {0, 50}, {110, 150}, {290, 185}, {385, 220}, {400, 25}
};
const int profileCount = sizeof(profile) / sizeof(profile[0]);

// Timers
unsigned long startTime;
bool running = false;

void setup() {
  Serial.begin(9600);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(LOGIC_PIN, OUTPUT);

  thermocouple.begin();
  thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);

  ovenPID.SetMode(AUTOMATIC);
  ovenPID.SetOutputLimits(0, 255);  // PWM range
  analogWrite(OUT_PIN, 0);
}

void loop() {
  if (!running && digitalRead(START_PIN) == LOW) {
    startTime = millis();
    running = true;
    Serial.println("Reflow started");
  }

  if (!running) return;

  // Read current temperature
  currentTemp = thermocouple.readThermocoupleTemperature();
  double elapsed = (millis() - startTime) / 1000.0;

  // Determine the current setpoint from profile
  targetTemp = getTargetTemp(elapsed);

  // Compute PID and apply output
  ovenPID.Compute();
  analogWrite(OUT_PIN, (int)heaterPower);

  // End condition
  if (elapsed >= profile[profileCount - 1][0]) {
    running = false;
    analogWrite(OUT_PIN, 0);
    Serial.println("Reflow complete");
  }

  // Optional debug output
  Serial.print("Time: "); Serial.print(elapsed);
  Serial.print(" s, Temp: "); Serial.print(currentTemp);
  Serial.print(" C, Target: "); Serial.print(targetTemp);
  Serial.print(" C, Output: "); Serial.println(heaterPower);

  delay(100);
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
