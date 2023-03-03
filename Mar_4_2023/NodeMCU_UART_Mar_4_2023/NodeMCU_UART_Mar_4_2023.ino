#define BLYNK_TEMPLATE_ID "TMPL9gdyPoKY"
#define BLYNK_TEMPLATE_NAME "BMS"
#define BLYNK_AUTH_TOKEN "XKeavZIa9xzla-ujnD9wPr9gLDcSrtoV"
#define BLYNK_PRINT Serial

#include "SerialTransfer.h"
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include "BlynkSimpleEsp8266.h"

SoftwareSerial nodeMCU(D2, D3);
SerialTransfer infoTransfer;

float test;
/*
const char *ssid =  "Minh's phon";
const char *pass =  "doanthuxemnao";
*/

const char *ssid = "USTH_Student";
const char *pass =  "usth2021!";

int powerState;

struct __attribute__((packed)) STRUCT {
  float x; //OCV
  float z1; //CC
  //float z2; // KF
  float p;//power
} infoStruct;

//BlynkTimer timer;

BLYNK_WRITE(V1){// POWER BUTTON, connect NO pin directly to the source
  powerState = param.asInt();
  //digitalWrite(D7, powerState);// read the power button state and write it
  // to pin D7
  //Serial.println(powerState); 
}

void setup() {
  Serial.begin(9600);
  nodeMCU.begin(9600);
  infoTransfer.begin(nodeMCU);
  // =============== WiFi interfacing ==================
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  //pinMode(D7, OUTPUT);
  //timer.setInterval(1000L, myTimerEvent); //Staring a timer called every 1s

}
// ======================================================
void loop() {
  Blynk.run();
  infoTransfer.sendDatum(powerState);
  if (infoTransfer.available()) {
    infoTransfer.rxObj(infoStruct);
    Blynk.virtualWrite(V3, infoStruct.x);
    Blynk.virtualWrite(V0, infoStruct.z1);
    Blynk.virtualWrite(V2, infoStruct.p);
    Serial.print(infoStruct.x);
    Serial.println(infoStruct.z1);
    Serial.println(infoStruct.p); 
  }
}
