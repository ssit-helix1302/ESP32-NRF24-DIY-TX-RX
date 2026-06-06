#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <nRF24L01.h>
#include <RF24.h>


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// Pins
#define vrx1 34
#define vry1 35
#define vrx2 32
#define vry2 33
#define sw1 27
#define sw2 16  
#define armled 13
//#define failled 17
#define nrfled 14
#define tx_batt_pin 36 


#define CE_PIN 4
#define CSN_PIN 5


RF24 radio(CE_PIN, CSN_PIN);
const byte location[6] = "00001";


typedef struct drone {
  int roll, pitch, yaw, throttle, aux2;
  byte arm, disarm;
} packet;


typedef struct telemetry {
  int rawDroneVoltage; // Received raw from RX
} telemPacket;


packet dataToRX;
telemPacket dataFromRX;


// Logic & Math Constants
const float drone_batt_ratio = 12.6; 
const float drone_batt_cal = 1.03; 
const float tx_batt_ratio = 2.0;   // 10k/10k local divider
const float tx_batt_cal = 1.065; 
const float ema_alpha = 0.05;


float smoothedDroneV = 0.0;
float smoothedTXV = 0.0;
//int currentRssi = 0;
unsigned long ackCount = 0;
//unsigned long lastRssiTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastSuccessTime = 0;
bool failsafeActive = false;
bool armstate = false;


int simpleChannel(int raw) {
  return map(raw, 0, 4095, 1000, 2000);
}


void processMath() {
  // Drone Battery Math
  float dPinV = (dataFromRX.rawDroneVoltage * 3.3) / 4095.0;
  float rawDV = dPinV * drone_batt_ratio * drone_batt_cal;
  //smoothedDroneV = rawDV ;
  if (smoothedDroneV < 10.4) smoothedDroneV = rawDV;
  else smoothedDroneV = (ema_alpha * rawDV) + ((1.0 - ema_alpha) * smoothedDroneV);


  // TX Battery Math
  float tPinV = (analogRead(tx_batt_pin) * 3.3) / 4095.0;
  float raw_TXV = tPinV * tx_batt_ratio * tx_batt_cal;
  if (smoothedTXV < 1.0) smoothedTXV = raw_TXV;
  else smoothedTXV = (ema_alpha * raw_TXV) + ((1.0 - ema_alpha) * smoothedTXV);
}


void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Line 1: Arm status and Local TX Battery
  display.setCursor(0,0);
  display.print(armstate ? "ARMED" : "DISARMED");
  display.setCursor(65, 0);
  display.print("TX: ");
  display.setCursor(85, 0);
  display.print(smoothedTXV, 2);
  display.print("V");


  // Line 2: Drone Voltage and RSSI
  display.setCursor(0, 25);
  if (failsafeActive) {
    display.setTextSize(2);
    display.println("LINK LOST");
  } else {
    processMath();
    display.setTextSize(1);
    display.print("Drone: ");
    display.print(smoothedDroneV/3, 2);
    display.println(" V");

    display.setTextSize(2);
    display.setCursor(35,45);
    display.print("HELIX");
    
    /*display.setCursor(0, 45);
    display.print("RSSI: ");
    display.print(currentRssi);
    display.print("%");*/
  }
  display.display();
}


void setup() {

  

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  //Wire.setClock(400000); // Speed up I2C for better stick response
  
  pinMode(tx_batt_pin, INPUT);
  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  pinMode(armled, OUTPUT); 
  //pinMode(failled, OUTPUT); 
  pinMode(nrfled, OUTPUT);


  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(76);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openWritingPipe(location);
}


void loop() {
  // 1. Human Input (Read & Map)
  dataToRX.throttle = 3000 - simpleChannel(analogRead(vry1));
  dataToRX.roll  = simpleChannel(analogRead(vrx2));
  dataToRX.pitch = 3000 - simpleChannel(analogRead(vry2));
  dataToRX.yaw   = simpleChannel(analogRead(vrx1));


  armstate = (digitalRead(sw1) == LOW);
  digitalWrite(armled, armstate);
  dataToRX.arm = armstate;
  dataToRX.disarm = !armstate;
  if(digitalRead(sw2)== LOW){
    dataToRX.aux2 = 2000;
  }
  else
    dataToRX.aux2 = 1000;
   


  // 2. Radio Exchange
  radio.stopListening();
  if (radio.write(&dataToRX, sizeof(dataToRX))) {
    lastSuccessTime = millis();
    failsafeActive = false;
    //digitalWrite(failled, LOW);
    //ackCount++; // Increment for RSSI calculation
    if (radio.isAckPayloadAvailable()) radio.read(&dataFromRX, sizeof(dataFromRX));
  }


  /*// 3. RSSI Math 
  if (millis() - lastRssiTime >= 1000) {
    currentRssi = map(ackCount, 0, 40, 0, 100);
    if (currentRssi > 100) currentRssi = 100;
    ackCount = 0;
    lastRssiTime = millis();
  }*/


  if (millis() - lastSuccessTime > 500) {
    failsafeActive = true;
    //digitalWrite(failled, HIGH);
  }
  
  digitalWrite(nrfled, radio.isChipConnected());


  // 4. Update UI
  if (millis() - lastDisplayUpdate > 100) {
    updateOLED();
    lastDisplayUpdate = millis();
  }
  delay(12);
}