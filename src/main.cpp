// Reflow Oven PID Control - Refactored and Simplified
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>
#include <TFT_eSPI.h>
#define FONT_SIZE    1
#define SERVO_PIN   14
// Button pins
#define PIN_BUTTON_1 0
#define PIN_BUTTON_2 1
#define PIN_BUTTON_3 2
#define DEBUG        1 // Select 1 for serial output
#define ONE_WIRE_BUS 8
#define MAX_CURRENT 15 // Total amp service from breaker
#define NUM_THERMOS  4 // Number of thermocouples
#define OUT_1_PIN   15
#define OUT_1_RES    7 // Resistance in Ohms
#if (NUM_THERMOS > 1)
#define OUT_2_PIN   16
#define OUT_2_RES    7 // Resistance in Ohms
#endif
#if (NUM_THERMOS > 2)
#define OUT_3_PIN   17
#define OUT_3_RES   11 // Resistance in Ohms
#endif
#if (NUM_THERMOS > 3)
#define OUT_4_PIN   18
#define OUT_4_RES   11 // Resistance in Ohms
#endif

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass oneWire reference to Dallas Temperature sensor object
DallasTemperature sensors(&oneWire);
DeviceAddress devices[NUM_THERMOS];
float temp[NUM_THERMOS], total_current = 0.0, heaterActual;
// SemaphoreHandle_t tempMutex; // protects access to temps
TaskHandle_t getTemps;

Servo doorServo;
// Published values for SG90 servos; adjust if needed
int minUs = 550;
int maxUs = 2350;

// PID control variables
double currentTemp, targetTemp, heaterPower, elementPower[NUM_THERMOS], elapsed;
unsigned long loopTime;
uint16_t dT = 0;
PID ovenPID(&currentTemp, &heaterPower, &targetTemp, 20, 1, 10, DIRECT);

// Cooling control variables
float coolingRate = -1.5; // °C/s
const float coolingTarget = 40.0; // Target temp to stop cooling
bool coolingActive = false;
unsigned long coolingStartTime;
float coolingStartTemp;

// Servo PID variables
double expectedTemp, servoOutput;
PID coolingPID(&currentTemp, &servoOutput, &expectedTemp, 2, 0.1, 1.5, REVERSE);

// TFT display
TFT_eSPI tft = TFT_eSPI();
#define GRAPH_HEIGHT 50
#define GRAPH_WIDTH 240
float graphData[GRAPH_WIDTH]; // Array to store temp values for plotting
float targetData[GRAPH_WIDTH]; // Array to store temp values for plotting
int graphIndex = 0, targetIndex = 0;

// Reflow profile segments: {time in sec, temp in C}
const float profile[][2] = {
  {0, 25}, {30+60, 100}, {120+60, 150}, {150+60, 183}, {210+60, 235},{240+60+300, 235}  // TS391AX
};
const int profileCount = sizeof(profile) / sizeof(profile[0]);

// Timers
unsigned long startTime;
bool running = false, startFlag = false;

// Don't want to include <algorithm.h>
template <typename T>
constexpr T clamp(T val, T min_val, T max_val) {
    return (val < min_val) ? min_val : (val > max_val) ? max_val : val;
}

void startCooling() {
    coolingActive = true;
    coolingStartTime = millis();
    coolingStartTemp = currentTemp;
    heaterPower = 0;
    targetTemp = 0;
    total_current = 0;
    for(int i = 0; i <NUM_THERMOS; i++)
        elementPower[i] = 0;
    startFlag = false;
    // turn off heating
    analogWrite(OUT_1_PIN, 0);
    #if (NUM_THERMOS > 1)
    analogWrite(OUT_2_PIN, 0);
    #endif
    #if (NUM_THERMOS > 2)
    analogWrite(OUT_3_PIN, 0);
    #endif
    #if (NUM_THERMOS > 3)
    analogWrite(OUT_4_PIN, 0);
    #endif
    Serial.println("Cooling started");
}

void IRAM_ATTR buttonISR1(){
    if (not startFlag){
        startFlag = 1;
    }
}

void IRAM_ATTR buttonISR2(){
    if(running){
        running = false;
        startCooling();
    } else if(coolingActive){
        coolingActive = false;
        targetTemp = 0;
        total_current = 0;
        analogWrite(OUT_1_PIN, 0);
        #if (NUM_THERMOS > 1)
        analogWrite(OUT_2_PIN, 0);
        #endif
        #if (NUM_THERMOS > 2)
        analogWrite(OUT_3_PIN, 0);
        #endif
        #if (NUM_THERMOS > 3)
        analogWrite(OUT_4_PIN, 0);
        #endif
    }
}

void IRAM_ATTR buttonISR3(){
    if(running){
        running = false;
        startCooling();
    } else if(coolingActive){
        coolingActive = false;
        targetTemp = 0;
        total_current = 0;
        analogWrite(OUT_1_PIN, 0);
        #if (NUM_THERMOS > 1)
        analogWrite(OUT_2_PIN, 0);
        #endif
        #if (NUM_THERMOS > 2)
        analogWrite(OUT_3_PIN, 0);
        #endif
        #if (NUM_THERMOS > 3)
        analogWrite(OUT_4_PIN, 0);
        #endif
    }
}

void getTempsTask(void* pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200);
    sensors.setWaitForConversion(false); // Non-blocking request
    while(true){
        sensors.requestTemperatures(); // Start async conversion
        // Wait *exactly* 200ms from last wake
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        for (int i = 0; i < NUM_THERMOS; i++)
            temp[i] = sensors.getTempC(devices[i]);
    }
}

uint16_t redBlueScale(float value, float min_val = 0, float max_val = 255) {
    value = clamp(value, min_val, max_val);
    float ratio = (value - min_val) / (max_val - min_val);

    uint8_t r = static_cast<uint8_t>(ratio * 255);
    uint8_t b = static_cast<uint8_t>((1.0f - ratio) * 255);

    return tft.color565(r, 100, b);
}

// Interpolates the target temperature from the profile
float getTargetTemp(float timeSec) {
  for (int i = 1; i < profileCount; i++) {
    float t0 = profile[i - 1][0], t1 = profile[i][0];
    float y0 = profile[i - 1][1], y1 = profile[i][1];
    if (timeSec <= t1) {
      float slope = (y1 - y0) / (t1 - t0);
      return y0 + slope * (timeSec - t0);
    }
  }
  return profile[profileCount - 1][1];
}

void manageCooling() {
    float timeSinceCooling = (millis() - coolingStartTime) * 0.001;
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

void balanceHeaterOutput(){
    // Heater resistances in ohms
    const float resistances[4] = {OUT_1_RES, OUT_2_RES, OUT_3_RES, OUT_4_RES};
    const float V = 120.0f;
    const float max_current = MAX_CURRENT;
    float balance_k = 0.2f;
    if(currentTemp > targetTemp) balance_k = 0.1;

    // Compute mean temperature
    float T_avg = 0.0f;
    for (int i = 0; i < 4; ++i)
        T_avg += temp[i];
    T_avg /= 4.0f;

    // Sensor pairs influence each heater
    // Contribution Matrix:
    const int sensor_pairs[4][2] = {
        {0, 3}, // Heater 0: between Sensor 0 and 3
        {0, 1}, // Heater 1: between Sensor 0 and 1
        {1, 2}, // Heater 2: between Sensor 1 and 2
        {2, 3}  // Heater 3: between Sensor 2 and 3
    };


    // Adjust PWM around global based on temp difference
    float raw_pwms[4];
    for (int i = 0; i < 4; ++i) {
        float e1 = T_avg - temp[sensor_pairs[i][0]];
        float e2 = T_avg - temp[sensor_pairs[i][1]];
        float avg_err = 0.5f * (e1 + e2); // positive = cooler than average
        float adjusted = heaterPower / 255 + balance_k * avg_err;
        raw_pwms[i] = clamp(adjusted, 0.0f, 1.0f) ;
    }

    // Compute current
    float total_current_raw = 0.0f;
    for (int i = 0; i < 4; ++i)
        total_current_raw += 1.25f * V * raw_pwms[i] / resistances[i]; // 125% rule for continuous loads

    // If current exceeds max, scale all outputs
    float scale = (total_current_raw > max_current) ? (max_current / total_current_raw) : 1.0f;

    // Clamp PWM and compute actual avg PWM and current
    total_current = 0.0;
    heaterActual = scale * heaterPower;
    for (int i = 0; i < 4; ++i){
        elementPower[i] = clamp(raw_pwms[i] * scale, 0.0f, 1.0f) * 255;
        total_current += 1.25f * V * elementPower[i] / (255 * resistances[i]); // 125% rule for continuous loads
    }
    #if DEBUG
    for(int i = 0; i < NUM_THERMOS; i++)
        Serial.printf("Balanced heater %i to %6.3f PWM\n",i,elementPower[i]);
    #endif
}

void setup() {
    Serial.begin(115200);
    pinMode(OUT_1_PIN, OUTPUT);
    #if (NUM_THERMOS > 1)
    pinMode(OUT_2_PIN, OUTPUT);
    #endif
    #if (NUM_THERMOS > 2)
    pinMode(OUT_3_PIN, OUTPUT);
    #endif
    #if (NUM_THERMOS > 3)
    pinMode(OUT_4_PIN, OUTPUT);
    #endif

    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_2, INPUT_PULLDOWN);
    pinMode(PIN_BUTTON_3, INPUT_PULLDOWN);
    // link the button 1 functions.
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_1), buttonISR1, RISING);
    // link the button 2 functions.
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_2), buttonISR2, RISING);
    // link the button 3 functions.
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_3), buttonISR3, RISING);

    // Start up the oneWire sensor library
    sensors.begin();
    #if DEBUG
    while(sensors.getDeviceCount() != NUM_THERMOS)
        Serial.println("Number of detected thermocouples mismatch!");
    Serial.println("Themocouples detected & initialized.");
    while(sensors.isParasitePowerMode())
        Serial.println("Parasitic power mode configured!");
    Serial.println("Normal power mode configured.");
    #endif
    for(int i = 0; i < NUM_THERMOS; i++){
        sensors.getAddress(devices[i], i);
        sensors.setResolution(devices[i], 4);
        #if DEBUG
        Serial.printf("Device %1i located at %i\n",i,devices[i]);
        #endif
    }
    xTaskCreatePinnedToCore(
        getTempsTask, // task function
        "getTempsTask", // name
        10000, // stack size
        NULL, // parameter
        1, // priority
        &getTemps, // task handle
        1 // core (0 or 1)
    );

    ovenPID.SetMode(AUTOMATIC);
    ovenPID.SetOutputLimits(0, 255);  // PWM range
    analogWrite(OUT_1_PIN, 0);
    #if (NUM_THERMOS > 1)
    analogWrite(OUT_2_PIN, 0);
    #endif
    #if (NUM_THERMOS > 2)
    analogWrite(OUT_3_PIN, 0);
    #endif
    #if (NUM_THERMOS > 3)
    analogWrite(OUT_4_PIN, 0);
    #endif

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
    tft.init();
    tft.setTextDatum(TL_DATUM);
    tft.setRotation(3);
    tft.setTextSize(FONT_SIZE);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(0, 5);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    // "It's me, ya boi"
    tft.printf("       Programmed by: Jacob Seman");
    tft.setCursor(0, 25);
    tft.printf("  ECEN4638 Controls Lab - Spring 2025");
    // Clear graph data
    memset(graphData, 0, sizeof(graphData));
}

void loop() {
    unsigned long now = millis();
    dT = now - loopTime;
    loopTime = now;

    // if (xSemaphoreTake(tempMutex, portMAX_DELAY)) {
        currentTemp = 0;
        for (int i = 0; i < NUM_THERMOS; i++){
            #if DEBUG
            if(running || coolingActive)Serial.printf("Thermocouple #%i: %6.3f\n",i,temp[i]);
            currentTemp += temp[i];
            #endif
        }
        currentTemp *= 0.25;
        // xSemaphoreGive(tempMutex);
    // }

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
        elapsed = (millis() - startTime) * 0.001;
        // Determine the current setpoint from profile
        targetTemp = getTargetTemp(elapsed);

        // Compute PID, balance heaters, and apply output
        ovenPID.Compute();
        balanceHeaterOutput();
        analogWrite(OUT_1_PIN, (int)elementPower[0]);
        #if (NUM_THERMOS > 1)
        analogWrite(OUT_2_PIN, (int)elementPower[1]);
        #endif
        #if (NUM_THERMOS > 2)
        analogWrite(OUT_3_PIN, (int)elementPower[2]);
        #endif
        #if (NUM_THERMOS > 3)
        analogWrite(OUT_4_PIN, (int)elementPower[3]);
        #endif

        // End condition
        if (!coolingActive && elapsed >= profile[profileCount - 1][0]) {
            running = false;
            heaterPower = 0;
            total_current = 0;
            for(int i = 0; i <NUM_THERMOS; i++)
                elementPower[i] = 0;
            startFlag = false;
            analogWrite(OUT_1_PIN, 0);
            #if (NUM_THERMOS > 1)
            analogWrite(OUT_2_PIN, 0);
            #endif
            #if (NUM_THERMOS > 2)
            analogWrite(OUT_3_PIN, 0);
            #endif
            #if (NUM_THERMOS > 3)
            analogWrite(OUT_4_PIN, 0);
            #endif
            Serial.println("Reflow complete");
            startCooling();
        }
    }
    #if DEBUG
    // Optional debug output
    Serial.print("Time: "); Serial.print(loopTime*0.001);
    Serial.print(" s, dT: "); Serial.print(dT*0.001);
    Serial.print(" s, Temp: "); Serial.print(currentTemp);
    Serial.print(" C, Target: "); Serial.print(coolingActive ? expectedTemp : targetTemp);
    Serial.print(" C, Output: "); Serial.println(coolingActive ? servoOutput : heaterPower);
    #endif

    for(int i = 1, j = graphIndex; i < GRAPH_WIDTH; i++){
        // Clear and redraw graph area
        tft.drawPixel(i - 1, GRAPH_HEIGHT - ((int)(graphData[i] * GRAPH_HEIGHT / profile[3][1])), TFT_BLACK);
        tft.drawPixel(i - 1, GRAPH_HEIGHT - ((int)(targetData[i] * GRAPH_HEIGHT / profile[3][1])), TFT_BLACK);
    }
    // Update graph data
    memmove(graphData, graphData + 1, (GRAPH_WIDTH - 1) * sizeof(float));
    memmove(targetData, targetData + 1, (GRAPH_WIDTH - 1) * sizeof(float));
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
    tft.printf("%14s%6i", coolingActive ? "Servo Angle:  " : "Heater Power: ",
                          coolingActive ? (int)servoOutput : (int)heaterPower);
    if(!coolingActive && running){
        tft.setCursor(78, GRAPH_HEIGHT + 10);
        tft.printf("%3i/",(int)heaterActual);
    }
    // Display temperatures
    tft.setCursor(0, GRAPH_HEIGHT + 30);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Avg Temp: %10.3f", currentTemp);
    // Display timestamp and timestep
    tft.setCursor(0, GRAPH_HEIGHT + 50);
    tft.printf("Target Temp: %7.3f", coolingActive ? expectedTemp : targetTemp);
    tft.setCursor(0, GRAPH_HEIGHT + 70);
    tft.printf("Timestamp: %9.3f, dT: %4.3f A: %4.2f", loopTime*0.001,dT*0.001,total_current);

    // Display temp and element values using layout:
    tft.fillRect(135, GRAPH_HEIGHT + 10, 165, 55, TFT_BLACK);
    tft.setCursor(135, GRAPH_HEIGHT + 10);
    tft.setTextColor(redBlueScale((int)elementPower[1]), TFT_BLACK);
    tft.printf("%  3i", (int)elementPower[1]);
    tft.setCursor(165, GRAPH_HEIGHT + 10);
    tft.setTextColor(redBlueScale((int)temp[1], 15, 400), TFT_BLACK);
    tft.printf("%  5.2f", temp[1]);
    tft.setCursor(205, GRAPH_HEIGHT + 10);
    tft.setTextColor(redBlueScale((int)elementPower[2]), TFT_BLACK);
    tft.printf("%  3i", (int)elementPower[2]);

    tft.setCursor(130, GRAPH_HEIGHT + 30);
    tft.setTextColor(redBlueScale((int)temp[0], 15, 400), TFT_BLACK);
    tft.printf("%  5.2f", temp[0]);
    tft.setCursor(195, GRAPH_HEIGHT + 30);
    tft.setTextColor(redBlueScale((int)temp[2], 15, 400), TFT_BLACK);
    tft.printf("%  5.2f", temp[2]);

    tft.setCursor(135, GRAPH_HEIGHT + 50);
    tft.setTextColor(redBlueScale((int)elementPower[0]), TFT_BLACK);
    tft.printf("%  3i", (int)elementPower[0]);
    tft.setCursor(165, GRAPH_HEIGHT + 50);
    tft.setTextColor(redBlueScale((int)temp[3], 15, 400), TFT_BLACK);
    tft.printf("%  5.2f", temp[3]);
    tft.setCursor(205, GRAPH_HEIGHT + 50);
    tft.setTextColor(redBlueScale((int)elementPower[3]), TFT_BLACK);
    tft.printf("%  3i", (int)elementPower[3]);
}
