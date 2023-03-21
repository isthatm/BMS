#include <SoftwareSerial.h>
#include <Adafruit_INA219.h>
#define S_IN_BUFF 10  // serial input buffer size
#define voltageSensorPin A0

uint8_t serial_in[S_IN_BUFF] = { '\0' };
//------------- Constants ----------------
const int relayPin = 8;
const int ledPin1 = 5;
const int ledPin2 = 7;

// Sensors: global variables are sent by sendESP()
//Current
Adafruit_INA219 ina219;
float accumCurrent;
int currentCounter = 0;

//=================================================

void setup() {
  Serial.begin(9600);         // with computer
  pinMode(relayPin, OUTPUT);  // relay
  pinMode(ledPin1, OUTPUT); 
  pinMode(ledPin2, OUTPUT); 
  Serial.setTimeout(1);
  while (!Serial) {}  // wait for serial interace to connect 
  if (!ina219.begin()) {
    while (1) { delay(10); }
  }
  digitalWrite(ledPin1, HIGH); // Good to go
  digitalWrite(relayPin, HIGH); 
}

//===================== MAIN ==========================

void loop() { 
  int numByte = 0;
  if (Serial.available()) {
    numByte = Serial.readBytes(serial_in, S_IN_BUFF - 1); // room for '\0'
    serial_in[numByte] = '\0';
    if (serial_in[0] == 49) {// ASCII character
      float avgOCV = getOCV();
      //digitalWrite(ledPin2, HIGH);
      Serial.print(avgOCV);
    } 
    else if (serial_in[0] == 50) {// measure sampling current
      getCurrent();
      Serial.println(currentCounter);// current is being sampled
      //digitalWrite(ledPin2, LOW);
    } 
    else if (serial_in[0] == 51) {// measure avg current
      float avgCurrent = getAvgCurrent();
      Serial.println(avgCurrent);
    }
  }
}

//=================================================

void getCurrent() {
  float current;
  current = ina219.getCurrent_mA() / 1000;
  accumCurrent += current;
  currentCounter++;
}

float getAvgCurrent(){
  float avgCurrent = accumCurrent / currentCounter;
  currentCounter = 0;
  return avgCurrent;
}

float getOCV() {
  digitalWrite(relayPin, LOW);  // disconnect the load for OCV measurement
  delay(5000);                  // rest for 3s -> battery comes to its steady state
  int counter = 0;
  float periodVoltageSum = 0;
  while (counter < 500) {
    int analog_value = analogRead(voltageSensorPin);
    float sampleOCV = (analog_value * 5.0) / 1024.0;
    periodVoltageSum += sampleOCV;
    counter++;
  }
  float OCV = periodVoltageSum / counter;
  digitalWrite(relayPin, HIGH);  // disconnect the load for OCV measurement
  return OCV;
}

