#include "SerialTransfer.h"
#include <SoftwareSerial.h>
#include <Adafruit_INA219.h>
#include "BasicLinearAlgebra.h"
using namespace BLA;

SoftwareSerial arduino(12, 11);  // Rx, Tx
SerialTransfer infoTransfer;

#define voltageSensorPin A3
//------------- Constants ----------------
const int relayPin = 8;

const int currentSamplingInterval = 300;
const int voltageSamplingInterval = 30000;
const int sensorAvgInterval = 3000;

//Sensors
Adafruit_INA219 ina219;
const int maxCapacity = 2.1;  // TODO: update this

//-------------Variables----------------
unsigned long currentMillis = 0;
unsigned long voltageMillis = 0;
unsigned long prevVoltageMillis = 0;
unsigned long prevCurrentMillis = 0;
unsigned long prevAvgMillis = 0;

// Sensors: global variables are sent by sendESP()
//Current
float avgCurrent;
float accumCurrent;
int sensorCounter = 0;
float totalCoulumbs = 0;
float initial_SOC = 23.63;

//Voltage
float OCV;  // for OCV
int updateTime = 0;
float accumVoltage;
float avgVoltage;  // for load voltage
int order = 5;
unsigned int loopTime;

// Power
int powerState = 1;

//====== Kalman Filter ======
#define Nstate 2                                 // length of the state vector
#define Nobs 2                                   // length of the measurement vector
Matrix<Nstate> last_x;                           //last_x(0): SOC; last_x(1): current
Matrix<Nstate, Nstate> last_P = { 1, 0, 0, 1 };  // varEst = 1
Matrix<Nstate, Nstate> K;

struct __attribute__((packed)) STRUCT {
  float x;
  float z1;  // Coulomb Counting
  //float z2; // Kalman Filter
  float p;  //Power
} infoStruct;

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
  OCV = measureOCV();
  infoStruct.x = OCV;
  digitalWrite(relayPin, HIGH);  // measure current
  int initSampling;
  float current;
  while (initSampling < 500) {
    current = ina219.getCurrent_mA() / 1000;
    accumCurrent += current;
    initSampling++;
  }
  avgCurrent = accumCurrent / initSampling;
  accumCurrent = 0;

  Serial.print("OCV:");  // in seconds
  Serial.println(OCV,4);
  Serial.print("; current:");  // in seconds
  Serial.println(avgCurrent);
  //last_x = {OCVtoSOC(OCV, polyOrder), current};
  delay(1000);
}

//=================================================

void loop() {
  currentMillis = millis();
  if (OCV < 2.8) {
    while (1) {
      digitalWrite(relayPin, LOW);
      Serial.println("Battery is completely discharged!");
      delay(3000);      
    }
  }
  
  //getPowerState();
  // if power state
  getOCV();
  getSensor();
}

//=================================================

void getPowerState() {
  if (infoTransfer.available()) {
    infoTransfer.rxObj(powerState);
    digitalWrite(relayPin, powerState);
  }
}

void getOCV() {
  if ((currentMillis - prevVoltageMillis >= voltageSamplingInterval) && (powerState == 1)) {
    updateTime = 1;
    unsigned long startTime = millis();
    digitalWrite(relayPin, LOW);  // disconnect the load for OCV measurement
    delay(5000);                  // rest for 3s -> battery comes to its steady state
    OCV = measureOCV();
    Serial.print("OCV:");
    Serial.println(OCV,4);
    digitalWrite(relayPin, HIGH);
    loopTime = millis() - startTime;                            // about 3930ms
    prevVoltageMillis += (voltageSamplingInterval + loopTime);  // !!! loop time should not be larger than
  }
}

void getSensor() {
  float current;
  float loadVoltage;
  //Collect the data given by sensor every sensorSamplingInterval(ms) and add them
  if (currentMillis - prevCurrentMillis >= currentSamplingInterval) {

    // Current Sensor
    current = ina219.getCurrent_mA() / 1000;

    //Voltage sensor
    loadVoltage = (analogRead(voltageSensorPin) * 5.0) / 1024.0;

    accumCurrent += current;
    accumVoltage += loadVoltage;

    prevCurrentMillis = millis();
    sensorCounter++;
  }

  // Average the accumulated sensor values every sensorAvgInterval(ms)
  if (sensorCounter == 10) {
    avgCurrent = (accumCurrent / sensorCounter);
    avgVoltage = (accumVoltage / sensorCounter);

    infoStruct.x = OCV;
    if (updateTime) {
      infoStruct.z1 = CC_SOC(millis() - prevAvgMillis - loopTime);  //CC
      Serial.print("SOC timer:");                                   // in seconds
      Serial.print(millis() - prevAvgMillis - loopTime);
      updateTime = 0;
    } else {
      infoStruct.z1 = CC_SOC(millis() - prevAvgMillis);  //CC
      Serial.print("SOC timer:");                        // in seconds
      Serial.print(millis() - prevAvgMillis);
    }
    //KF(ocvSOC, avgCurrent, sensorAvgInterval);
    //infoStruct.z2 = last_x(0);
    infoStruct.p = -(avgCurrent * avgVoltage);  //Power
    Serial.print("; Current:");                 // amps
    Serial.print(avgCurrent);
    Serial.print("; Power:");  // W
    Serial.print(infoStruct.p);
    Serial.print("; SOC:");
    Serial.println(infoStruct.z1);

    //infoTransfer.sendDatum(infoStruct);  // send to nodeMCU

    sensorCounter = 0;
    accumCurrent = 0;
    accumVoltage = 0;
    prevAvgMillis = millis();
  }
}

float OCVtoSOC(float OCV) {
  float ocvSOC = 0;
  int i = 0;
  if (OCV < 2.70) {
    OCV = 2.70;
  } else if (OCV > 4.20) {
    OCV = 4.20;
  }
  float coef[order + 1] = { 1, -2, 3, 4 };  //TODO: enter coefficient here
  while (i <= order) {
    ocvSOC += (coef[i] * pow(OCV, order - i));
    i++;
  }
  return ocvSOC;
}

float CC_SOC(int deltaT) {
  totalCoulumbs = totalCoulumbs + avgCurrent * (deltaT) / 1000;
  float TotalAh = totalCoulumbs / 3600.0;
  float CCSoC = initial_SOC + (TotalAh / maxCapacity) * 100;
  return CCSoC;
}

void KF(float measuredSOC, float measuredCurrent, int deltaT) {
  int start = millis();
  Matrix<Nstate> x_measurement = { measuredSOC, measuredCurrent };  //state vector
  Matrix<Nstate> x;                                                 //state vector
  Matrix<Nstate, Nstate> P;

  Matrix<Nstate, Nstate> R = { 3, 0, 0, 4.71 * pow(10, -6) };  // measurement noise
  Matrix<Nstate, Nstate> H = { 1, 0, 0, 1 };                   // Observation matrix
  Matrix<Nstate, Nstate> I = { 1, 0, 0, 1 };

  //Prediction
  Matrix<Nstate, Nstate> F = { 1, -100 * (deltaT / maxCapacity), 0, 1 };  //Transformation matrix
  x = F * last_x;                                                         // predict the state
  P = F * last_P * ~F;                                                    // predict uncertainty
  P(0, 1) = 0;
  P(1, 0) = 0;

  //Correction
  Matrix<Nstate, Nstate> S = P + R;
  Matrix<Nstate, Nstate> invS = S;
  bool is_nonsingular = Invert(invS);

  //Update
  K = P * invS;  // Kalman gain
  last_x = x + K * (x_measurement - x);
  last_P = (I - (K * H)) * P * ~(I - (K * H)) + (K * R * ~K);
  //last_P = (I - K) * P;
  /*
  Serial.print("start: ");
  Serial.println(start);
  Serial.print("end:");
  Serial.println(millis());
  */
}

/*
void KF(float measuredSOC, float measuredCurrent, int deltaT) {// original version
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

float OCVtoSOC(float measuredOCV, int order){
  BLA::Matrix<3> coef = {1,2,3};
  float ocvSOC = 0;
  int i = 0;

  if (measuredOCV < 2.5){// TODO: check OCV when SOC = 0%?
    measuredOCV = 2.5;
  }
  if (measuredOCV > 3.546){
    measuredOCV = 3.546;
  }
  while (order - i >= 0){
    ocvSOC = ( measuredOCV**(order - i) )*coef(i)  ;
    i++;
  }

  return ocvSOC
}
*/

float measureOCV() {
  int counter = 0;
  float periodVoltageSum = 0;
  while (counter < 500) {  // read for 1000 samples (approx 1000ms), TODO: change to 5000 later
    int analog_value = analogRead(voltageSensorPin);
    float sampleOCV = (analog_value * 5.0) / 1024.0;  // multiply by 5.128 for voltage module
    periodVoltageSum += sampleOCV;
    counter++;
  }
  float yourOCV = periodVoltageSum / counter;
  return yourOCV;
}

float testOCV() {
  int analog_value = analogRead(voltageSensorPin);
  float sampleOCV = (analog_value * 5.0) / 1024.0;  // multiply by 5.128 for voltage module
  return sampleOCV;
}
