// Reflow Oven PID Control - Refactored and Simplified
#include <PID_v1.h>
#include <Adafruit_MAX31856.h>
#include <ESP32Servo.h>
#if FEATHER_S3_REV
    #include <Adafruit_ST7789.h>
    #define TFT_CYAN ST77XX_CYAN
    #define TFT_BLACK ST77XX_BLACK
    #define TFT_WHITE ST77XX_WHITE
    #define TFT_GREEN ST77XX_GREEN
    #define TFT_RED ST77XX_RED
    #define TFT_ORANGE ST77XX_ORANGE
    #define FONT_SIZE    1
    #define DRDY_PIN    15
    #define OUT_PIN     16
    #define START_PIN   17
    #define SERVO_PIN   18
    // Button pins
    #define PIN_BUTTON_1 0
    #define PIN_BUTTON_2 1
    #define PIN_BUTTON_3 2
#elif FEATHER_S3_REV_ESPI
    #include <TFT_eSPI.h>
    #define FONT_SIZE    1
    #define DRDY_PIN    15
    #define OUT_PIN     16
    #define START_PIN   17
    #define SERVO_PIN   18
    // Button pins
    #define PIN_BUTTON_1 0
    #define PIN_BUTTON_2 1
    #define PIN_BUTTON_3 2
#else
    #include <TFT_eSPI.h>
    #define FONT_SIZE    2
    #define DRDY_PIN    21
    #define OUT_PIN     16
    #define START_PIN    4
    #define SERVO_PIN    1
    // Button pins
    #define PIN_BUTTON_1 0
    #define PIN_BUTTON_2 14
#endif

#define DEBUG 0 // Select 1 for serial output


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


// TFT display
#if FEATHER_S3_REV
    Adafruit_ST7789 tft = Adafruit_ST7789(42,40,41);
    #define GRAPH_HEIGHT 50
    #define GRAPH_WIDTH 240
#elif FEATHER_S3_REV_ESPI
    TFT_eSPI tft = TFT_eSPI();
    #define GRAPH_HEIGHT 50
    #define GRAPH_WIDTH 240
#else
    TFT_eSPI tft = TFT_eSPI();
    #define GRAPH_HEIGHT 70
    #define GRAPH_WIDTH 320
#endif
double graphData[GRAPH_WIDTH]; // Array to store pressure values for plotting
double targetData[GRAPH_WIDTH]; // Array to store pressure values for plotting
int graphIndex = 0, targetIndex = 0;

// Reflow profile segments: {time in sec, temp in C}
const double profile[][2] = {
  {0, 50}, {1.10, 150}, {2.90, 185}, {3.85, 220}, {4.00, 25}
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

void IRAM_ATTR buttonISR2(){
    running = 0;
    analogWrite(OUT_PIN, 0);
    startFlag = 0;
}

#if FEATHER_S3_REV || FEATHER_S3_REV_ESPI
void IRAM_ATTR buttonISR3(){
    running = 0;
    analogWrite(OUT_PIN, 0);
    startFlag = 0;
}
#endif

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
#if DEBUG
        Serial.println("Cooling complete");
#endif
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(OUT_PIN, OUTPUT);

    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
#if FEATHER_S3_REV || FEATHER_S3_REV_ESPI
    pinMode(PIN_BUTTON_2, INPUT_PULLDOWN);
    pinMode(PIN_BUTTON_3, INPUT_PULLDOWN);
#else
    pinMode(PIN_BUTTON_2, INPUT_PULLUP);
#endif
    // link the button 1 functions.
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_1), buttonISR1, RISING);
    // link the button 2 functions.
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_2), buttonISR2, RISING);
#if FEATHER_S3_REV || FEATHER_S3_REV_ESPI
    // link the button 3 functions.
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_3), buttonISR3, RISING);
#endif

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

    // Initialize TFT
#if FEATHER_S3_REV
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
    tft.init(135,240);
    #if DEBUG
        Serial.println("TFT init complete");
    #endif
#elif FEATHER_S3_REV_ESPI
    tft.init();
    tft.setTextDatum(TL_DATUM);
    tft.setRotation(3);
#else
    tft.init();
    tft.setTextDatum(TL_DATUM);
    tft.setRotation(1);
#endif
    tft.setTextSize(FONT_SIZE);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(0, 5);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    // "It's us, ya bois"
    tft.printf("Prog'd: %18s", "Jake, Josh, Kyle");
    tft.setCursor(0, 25);
    tft.printf("%26s", "ECEN4638 Controls Lab");
    tft.setCursor(0, 45);
    tft.printf("%26s", "Spring 2025");
    // Clear graph data
    memset(graphData, 0, sizeof(graphData));
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
        tft.fillScreen(TFT_BLACK);
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
#if DEBUG
    // Optional debug output
    Serial.print("Time: "); Serial.print(loopTime);
    Serial.print(" s, dT: "); Serial.print(dT);
    Serial.print(" s, Temp: "); Serial.print(currentTemp);
    Serial.print(" C, Target: "); Serial.print(targetTemp);
    Serial.print(" C, Output: "); Serial.println(heaterPower);
#endif

    for(int i = 1, j = graphIndex; i < GRAPH_WIDTH; i++){
        // Clear and redraw graph area
        tft.drawPixel(i - 1, GRAPH_HEIGHT - ((int)(graphData[i] * GRAPH_HEIGHT / profile[3][1])), TFT_BLACK);
        tft.drawPixel(i - 1, GRAPH_HEIGHT - ((int)(targetData[i] * GRAPH_HEIGHT / profile[3][1])), TFT_BLACK);
    }
    // Update graph data
    memmove(graphData, graphData + 1, (GRAPH_WIDTH - 1) * sizeof(double));
    memmove(targetData, targetData + 1, (GRAPH_WIDTH - 1) * sizeof(double));
    graphData[GRAPH_WIDTH - 1] = currentTemp;
    targetData[GRAPH_WIDTH - 1] = coolingActive ? expectedTemp : targetTemp;
    // Draw data
    for(int i = 1; i < GRAPH_WIDTH; i++){
        tft.drawPixel(i - 1, GRAPH_HEIGHT - ((int)(graphData[i] * GRAPH_HEIGHT / profile[3][1])), TFT_GREEN);
        tft.drawPixel(i - 1, GRAPH_HEIGHT - ((int)(targetData[i] * GRAPH_HEIGHT / profile[3][1])), coolingActive ? TFT_CYAN : TFT_ORANGE);
    }

    tft.setTextSize(FONT_SIZE);
    // Display solenoid status
    tft.setCursor(0, GRAPH_HEIGHT + 10);
    tft.setTextColor(startFlag ? TFT_GREEN : coolingActive ? TFT_CYAN : TFT_RED, TFT_BLACK);
    tft.printf("%14s%12i",coolingActive ? "Servo Angle:  " : "Heater Power: ",coolingActive ? (int)servoOutput : (int)heaterPower);
    // Display pressure value
    tft.setCursor(0, GRAPH_HEIGHT + 30);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Current Temp: %12.3f", currentTemp);
    // Display timestamp and timestep
    tft.setCursor(0, GRAPH_HEIGHT + 50);
    tft.printf("Target Temp: %13.3f", coolingActive ? expectedTemp : targetTemp);
    tft.setCursor(0, GRAPH_HEIGHT + 70);
#if FEATHER_S3_REV || FEATHER_S3_REV_ESPI
    tft.printf("Timestamp: %15.3f, %4.3f", loopTime,dT);
#else
    tft.printf("Timestamp: %8.3f, %4.3f", loopTime,dT);
#endif
    // delay(100);
}
