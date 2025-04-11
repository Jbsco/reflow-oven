#include <PID_v1.h>
#include <Adafruit_MAX31856.h>

#define DRDY_PIN    5
#define OUT_PIN     10
#define START_PIN   4
#define LOGIC_PIN   2

Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(9);

const int len = 10;
double temperature[len] = {0};
const int freq = 50;
//double av_temp = 0;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=20, Ki=0, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double pSet, pIn, pOut;
double pKp = 25, pKi = 0, pKd = 1;
PID preHeat(&pIn, &pOut, &pSet, pKp, pKi, pKd, DIRECT);

// time in secs, temperature in C
const double setpoint_0[2] = {0, 50};     // {0, 50}
const double setpoint_1[2] = {110, 150};   // {90, 150}
const double setpoint_2[2] = {290, 185};   // {185, 185}
const double setpoint_3[2] = {385, 220};   // {225, 220}
const double setpoint_4[2] = {400, 25};    // {240, 25}
double slope_1, slope_2, slope_3, slope_4;
double y_int_1, y_int_2, y_int_3, y_int_4;
long int start_time;

bool start = 0;
bool ready = 0;
bool pressed = 0;
bool decrease = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(500);
  //initialize the variables we're linked to
  slope_1 = (setpoint_0[1] - setpoint_1[1])/(setpoint_0[0] - setpoint_1[0]);
  slope_2 = (setpoint_1[1] - setpoint_2[1])/(setpoint_1[0] - setpoint_2[0]);
  slope_3 = (setpoint_2[1] - setpoint_3[1])/(setpoint_2[0] - setpoint_3[0]);
  slope_4 = (setpoint_3[1] - setpoint_4[1])/(setpoint_3[0] - setpoint_4[0]);

  y_int_1 = setpoint_1[1] - slope_1*setpoint_1[0];
  y_int_2 = setpoint_2[1] - slope_2*setpoint_2[0];
  y_int_3 = setpoint_3[1] - slope_3*setpoint_3[0];
  y_int_4 = setpoint_4[1] - slope_4*setpoint_4[0];
  Setpoint = 29;


  pinMode(OUT_PIN, OUTPUT); // Must use D10 for slower PWM
  pinMode(LOGIC_PIN, OUTPUT);
  pinMode(START_PIN, INPUT);

  digitalWrite(LOGIC_PIN, HIGH);

  maxthermo.begin();
  delay(250);
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  preHeat.SetMode(AUTOMATIC);

  maxthermo.setConversionMode(MAX31856_CONTINUOUS);
}

void loop()
{
  if(start & ready){
    if((millis()-start_time)/1000.0 < setpoint_1[0]){
      Setpoint = slope_1*((millis()-start_time)/1000.0) + y_int_1;
    }
    else if((millis()-start_time)/1000.0 < setpoint_2[0]){
      Setpoint = slope_2*((millis()-start_time)/1000.0) + y_int_2;
    }
    else if((millis()-start_time)/1000.0 < setpoint_3[0]){
      Setpoint = slope_3*((millis()-start_time)/1000.0) + y_int_3;
    }
    //else if((millis()-start_time)/1000.0 < setpoint_4[0]){
      //Setpoint = slope_4*((millis()-start_time)/1000.0) + y_int_4;
    //}
    else if(((millis()-start_time)/1000.0 > setpoint_3[0]) & (temperature[len-1] < Setpoint) & !decrease){
      Setpoint = setpoint_3[1];
      //decrease = 1;
    }
    else if(((millis()-start_time)/1000.0 > setpoint_3[0]) & (temperature[len-1] < Setpoint) & decrease){
      Setpoint = 25;
    }
    
    for(int i = 1; i < len; i++){
      temperature[i-1] = temperature[i];
    }
    temperature[len-1] = read_temp();

    Input = temperature[len-1];
    //Input = average(len, temperature);
    Serial.print(Input);
    Serial.print("\t");

    myPID.Compute();
    Serial.print(Output);
    Serial.print("\t");
    Serial.println(Setpoint);
    myPwm(Output, freq);
    if(Input >= setpoint_3[1]){
      decrease = 1;
    }
  }
  else{
    pSet = 52;
    pIn = read_temp();
    preHeat.Compute();
    Serial.print(pIn);
    Serial.print("\t");
    Serial.println(pOut);
    myPwm(pOut, freq);
    if(pIn > 49){
      ready = 1;
    }
    if((digitalRead(START_PIN) == 1) & !pressed & ready){
      start = 1;
      start_time = millis();
      pressed = 1;
    }
  }
  
  delay(100);
}

void myPwm(unsigned char duty, float freq) {
  TCCR1A = 0x21;
  TCCR1B = 0x14;
  OCR1A = 0x7A12 / freq;
  OCR1B = OCR1A * (duty / 255.0);
}

double average(int num, double *array){
  double sum = 0;
  for(int i = 0; i < num; i++){
    sum += array[i];
  }
  double average = sum/num;
  return average;
}

double read_temp(){
  int count = 0;
  while(digitalRead(DRDY_PIN)){
    if (count++ > 200) {
      count = 0;
    }
  }
  return maxthermo.readThermocoupleTemperature();
}


