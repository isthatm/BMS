#include "SerialTransfer.h"
#include <SoftwareSerial.h>
#include "SimpleKalmanFilter.h"
#include <Adafruit_INA219.h>
#include "BasicLinearAlgebra.h"

SoftwareSerial arduino(12, 11);  // Rx, Tx
SerialTransfer infoTransfer;

#define voltageSensorPin A3
#define currentSensorPin A2
//------------- Constants ----------------
const int relayPin = 8;

const int currentSamplingInterval = 100;   // 3s in actual operation
const int voltageSamplingInterval = 5000;  // 3s in actual operation
const int sensorAvgInterval = 1000;        // 3s in actual operation
const int ESPInterval = 5000;              // 5mins in actual operation

//Sensors
const float sensitivity = 0.1;
const float offsetVoltage = 2.5;
Adafruit_INA219 ina219;
/*
  SimpleKalmanFilter(e_mea, e_est, q);
  e_mea: Measurement Uncertainty // from excel file
  e_est: Estimation Uncertainty
  q: Process Noise
*/
float e1 = 1.00, e2 = 1.00, q = 1.0;
SimpleKalmanFilter currentKF(e1, e2, q);

const int maxCapacity = 25;

//-------------Variables----------------
unsigned long currentMillis = 0;
unsigned long voltageMillis = 0;
unsigned long prevVoltageMillis = 0;
unsigned long prevCurrentMillis = 0;
unsigned long prevAvgMillis = 0;
unsigned long prevESPMillis = 0;
int loopTime;
int calTime;
int addTime;  //logic variable for adding extra time

// Sensors: global variables are sent by sendESP()
//Current
float avgCurrent;
float accumCurrent;
int sensorCounter = 0;
float totalCoulumbs = 0;
float initial_SOC = 100;
float Rload = 2.6;  // red wire = 0.8ohm, black = 0.8ohm
float initCurrent;

//Voltage
float OCV;  // for OCV
float accumVoltage;
float avgVoltage;  // for load voltage
float initOCV;
// Power
int powerState;
int powerFlag = 1;

//====== Kalman Filter ======
#define Nstate 2                                      // length of the state vector
#define Nobs 2                                        // length of the measurement vector
BLA::Matrix<Nstate> last_x;                           //last_x(0): SOC; last_x(1): current
BLA::Matrix<Nstate, Nstate> last_P = { 1, 0, 0, 1 };  // varEst = 1

struct __attribute__((packed)) STRUCT {
  float x;
  //float y;
  float z1;
  float p;  //Power
} infoStruct;

int OCVcounter = 0;

//=================================================

void setup() {
  Serial.begin(9600);   // with computer
  arduino.begin(9600);  // with NodeMCU
  infoTransfer.begin(arduino);
  pinMode(relayPin, OUTPUT);  // relay

  while (!Serial) {
    delay(1);
  }
  Serial.print("Hello!");
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // ======== KF initialization ========
  //TODO: write the whole procedure for OC and current measurement here
  digitalWrite(relayPin, LOW);  // disconnect the load for OCV measurement
  initOCV = measureOCV();
  digitalWrite(relayPin, HIGH);  // measure current
  int initSampling;
  float current;
  while (initSampling < 1000) {
    current = ina219.getCurrent_mA() / 1000;
    accumCurrent = +current;
    initSampling++;
  }
  initCurrent = accumCurrent / initSampling;
  accumCurrent = 0;
}

//=================================================

void loop() {
  currentMillis = millis();
  /*
    Serial.print("Time elapsed:");
    Serial.println(currentMillis);
  */
  getPowerState();
  //Serial.println(powerState);
  if (powerState) {
    addTime = 0;
    getOCV();
    getSensor();
    //sendESP();
    //testOCV();
  }
    Serial.print("Voltage;");
    Serial.print(infoStruct.x);
    Serial.print("V;");
    Serial.print("Current;");
    Serial.print(avgCurrent);
    Serial.print("A;");
    Serial.print("Power;");
    Serial.print(infoStruct.p);
    Serial.print("W;");
    Serial.print("SOC;");
    Serial.print(infoStruct.z1);
    Serial.print("%");
    Serial.println("");
    delay(1000);
}

//=================================================

void getPowerState() {
  if (infoTransfer.available()) {
    infoTransfer.rxObj(powerState);
    //Serial.println(powerState);
    if (powerFlag) {
      digitalWrite(relayPin, powerState);
    }
  }
}

void getOCV() {  // !!! add the parameter to be sent into infoStruct
  if ((currentMillis - prevVoltageMillis >= voltageSamplingInterval) && (powerState == 1)) {
    /*
      Serial.print("Measure OCV: ");
      Serial.println(powerState); */
    int startTime = millis();
    digitalWrite(relayPin, LOW);  // disconnect the load for OCV measurement
    delay(3000);                  // rest for 3s -> battery comes to its steady state
    OCV = measureOCV();
    digitalWrite(relayPin, HIGH);
    loopTime = millis() - startTime;                            // about 3930ms
    prevVoltageMillis += (voltageSamplingInterval + loopTime);  // !!! loop time should not be larger than
    addTime = 1;
    OCVcounter++;
    Serial.print("called OCV:");
    Serial.println(OCVcounter);
  }
}

void getSensor() {
  float current;
  float loadVoltage;
  //Collect the data given by sensor every sensorSamplingInterval(ms) and add them
  if (currentMillis - prevCurrentMillis >= currentSamplingInterval) {

    // Current Sensor
    float testcurrent = ina219.getCurrent_mA() / 1000;
    if (testcurrent == testcurrent) {
      current = testcurrent;
      //Serial.print("Test!!!!!!!!!!!!");
    } else {
      current = 0;
      //Serial.print("Detect!!!!!!!!!!!!");
    }

    //Voltage sensor
    loadVoltage = (analogRead(voltageSensorPin) * 5.0) / 1024.0;
    //}

    accumCurrent += current;
    accumVoltage += loadVoltage;

    /*
    if (addTime) {
      prevCurrentMillis += (currentSamplingInterval);
    }
    else {
      prevCurrentMillis += currentSamplingInterval;// add extra time if interval < OCV interval to prevent add-up measurements
    }
    */
    prevCurrentMillis = millis();
    sensorCounter++;
  }

  // Average the accumulated sensor values every sensorAvgInterval(ms)
  if (sensorCounter == 10) {
    //int calStart = millis(); // this function calculates SOC so the delay is more significant
    //float randNum = random(0, 5);
    avgCurrent = (accumCurrent / sensorCounter);  //TODO: remove randNum
    avgVoltage = (accumVoltage / sensorCounter);

    infoStruct.x = OCV;
    //infoStruct.y = avgCurrent;
    infoStruct.z1 = CC_SOC();                    //SOC
    infoStruct.p = -(avgCurrent * avgVoltage);  //Power
    infoTransfer.sendDatum(infoStruct);         // send to nodeMCU

    sensorCounter = 0;
    accumCurrent = 0;
    accumVoltage = 0;
    //int calTime = millis() - calStart;
    //prevAvgMillis += (sensorAvgInterval + calTime);
  }
}

float measureOCV() {
  int counter = 0;
  float periodVoltageSum = 0;
  while (counter < 1000) {  // read for 1000 samples (approx 1000ms), TODO: change to 5000 later
    int analog_value = analogRead(voltageSensorPin);
    float sampleOCV = (analog_value * 5.0) / 1024.0;  // multiply by 5.128 for voltage module
    periodVoltageSum += sampleOCV;
    counter++;
  }
  float yourOCV = periodVoltageSum / counter;
  return yourOCV;
}

float CC_SOC() {  //TODO: initial coulomb?
  totalCoulumbs = totalCoulumbs + avgCurrent * (sensorAvgInterval) / 1000;
  float TotalAh = totalCoulumbs / 3600.0;
  float CCSoC = initial_SOC + (TotalAh / maxCapacity) * 100;  // FIXED should be Soc not initial
  // SOH =(max Ah of battery / new batt max AH)*100;
  return CCSoC;
}

void KF(float measuredSOC, float measuredCurrent, int deltaT) {
  float meaNoise = 0.5;
  BLA::Matrix<Nstate> x_measurement = { measuredSOC, measuredCurrent };  //state vector
  BLA::Matrix<Nstate> x;                                                 //state vector
  BLA::Matrix<Nstate, Nstate> P;

  BLA::Matrix<Nstate, Nstate> R = { meaNoise, 0, 0, meaNoise };  // measurement noise
  BLA::Matrix<Nstate, Nstate> H = { 1, 0, 0, 1 };                // Observation matrix
  BLA::Matrix<Nstate, Nstate> I = { 1, 0, 0, 1 };                // measurement noise

  //Prediction
  BLA::Matrix<Nstate, Nstate> F = { 1, -1 * (deltaT / maxCapacity) * 100, 0, 1 };  //Transformation matrix
  x = F * last_x;                                                                  // predict the state
  P = F * last_P * ~F;                                                             // predict uncertainty

  //Correction
  BLA::Matrix<Nstate, Nstate> S = (H * last_P * ~H) + R;
  BLA::Matrix<Nstate, Nstate> invS = S;
  bool is_nonsingular = Invert(invS);
  BLA::Matrix<Nstate, Nstate> K;

  //Update
  K = last_P * ~H * invS;  // Kalman gain
  last_x = x + K * (x_measurement - (H * x));
  last_P = (I - K * H) * P * ~(I - K * H) + (K * R * ~K);
}


void testOCV() {  // !!! add the parameter to be sent into infoStruct
  int analog_value = analogRead(voltageSensorPin);
  float OCV = (analog_value * 5.0) / 1024.0;
  //float filteredOCV = currentKF.updateEstimate(OCV);
  Serial.println("open OCV = ");
  Serial.println(OCV, 4);
  delay(1000);
}