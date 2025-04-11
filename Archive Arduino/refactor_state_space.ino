// Reflow Oven Control with State-Space Control for Heating and Cooling
#include <Adafruit_MAX31856.h>
#include <Servo.h>

#define DRDY_PIN    5
#define OUT_PIN     10
#define START_PIN   4
#define LOGIC_PIN   2
#define SERVO_PIN   6

// Uncomment to enable model identification mode
//#define MODEL_IDENTIFICATION

#include <BasicLinearAlgebra.h>
using namespace BLA;

Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(9);
Servo doorServo;

// Oven control variables
double currentTemp, targetTemp;
bool coolingActive = false;
unsigned long coolingStartTime;
double coolingStartTemp;

// Reflow profile segments: {time in sec, temp in C}
const double profile[][2] = {
  {0, 50}, {110, 150}, {290, 185}, {385, 220}, {400, 25}
};
const int profileCount = sizeof(profile) / sizeof(profile[0]);

// Timers
unsigned long startTime;
bool running = false;

// State-space model: x' = Ax + Bu, y = Cx + Du
// Unified model for both heating and cooling
BLA::Matrix<1,1> x = {0};   // Temperature state
BLA::Matrix<1,1> A = {0.95};  // Decay/retention factor
BLA::Matrix<1,1> B_heat = {0.1};   // Heating influence
BLA::Matrix<1,1> B_cool = {0.05};  // Cooling influence
BLA::Matrix<1,1> C = {1.0};
BLA::Matrix<1,1> D = {0.0};
BLA::Matrix<1,1> K_heat = {8};    // Heating gain
BLA::Matrix<1,1> K_cool = {10};   // Cooling gain
BLA::Matrix<1,1> r = {0};         // Reference temp

const double coolingRate = -3.0;       // °C/s
const double coolingTarget = 50.0;     // End temp for cooling

void setup() {
  Serial.begin(9600);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(LOGIC_PIN, OUTPUT);

  thermocouple.begin();
  thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);
  analogWrite(OUT_PIN, 0);

  doorServo.attach(SERVO_PIN);
  doorServo.write(0);
}

void loop() {
  if (!running && digitalRead(START_PIN) == LOW) {
    startTime = millis();
    running = true;
    Serial.println("Reflow started");
  }

  if (!running) return;

  currentTemp = thermocouple.readThermocoupleTemperature();
  double elapsed = (millis() - startTime) / 1000.0;

  if (coolingActive) {
    manageCooling();
  } else {
    targetTemp = getTargetTemp(elapsed);
    manageHeating();
  }

  if (!coolingActive && elapsed >= profile[profileCount - 1][0]) {
    startCooling();
  }

  Serial.print("Time: "); Serial.print(elapsed);
  Serial.print(" s, Temp: "); Serial.print(currentTemp);
  Serial.print(" C, Target: "); Serial.print(targetTemp);
  Serial.print(" C\n");

  delay(100);
}

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

void manageHeating() {
  r(0) = targetTemp;
  BLA::Matrix<1,1> u = -K_heat * (x - r);
  u(0) = constrain(u(0), 0, 255);
  analogWrite(OUT_PIN, (int)u(0));
  x = A * x + B_heat * u;
}

void startCooling() {
  coolingActive = true;
  coolingStartTime = millis();
  coolingStartTemp = currentTemp;
  analogWrite(OUT_PIN, 0);
  x(0) = currentTemp;
  Serial.println("Cooling started");
}

void manageCooling() {
  double t = (millis() - coolingStartTime) / 1000.0;
  r(0) = coolingStartTemp + coolingRate * t;

#ifdef MODEL_IDENTIFICATION
  static int step = 0;
  int angle = (step / 100) % 5 * 20;
  doorServo.write(angle);
  step++;
  Serial.print("ID Mode, t="); Serial.print(t);
  Serial.print(", Servo="); Serial.print(angle);
  Serial.print(", Temp="); Serial.println(currentTemp);
#else
  BLA::Matrix<1,1> u = -K_cool * (x - r);
  u(0) = constrain(u(0), 0, 90);
  doorServo.write((int)u(0));
  x = A * x + B_cool * u;
#endif

  if (currentTemp <= coolingTarget) {
    coolingActive = false;
    running = false;
    doorServo.write(0);
    Serial.println("Cooling complete");
  }
}
