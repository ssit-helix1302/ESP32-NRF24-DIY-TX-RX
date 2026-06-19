#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 4
#define CSN_PIN 5
#define IBUS_TX 26     
#define VOLTAGE_PIN 36

RF24 radio(CE_PIN, CSN_PIN);
const byte location[5] = "F450";

typedef struct __attribute__((packed)) drone {
  int32_t roll, pitch, yaw, throttle, aux2;
  uint8_t arm, disarm;
} packet;

typedef struct __attribute__((packed)) telemetry {
  int32_t rawDroneVoltage;
} telemPacket;

packet dataFromTX;
telemPacket dataToTX;

unsigned long lastRecvTime = 0;
uint16_t channels[14];

unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 10; 
bool wasConnected = true; 

void sendIBUS() {
  uint8_t packet[32];
  packet[0] = 0x20; packet[1] = 0x40;
  uint16_t checksum = 0xFFFF - packet[0] - packet[1];
  for (int i = 0; i < 14; i++) {
    packet[2 + i*2] = channels[i] & 0xFF;
    packet[3 + i*2] = (channels[i] >> 8) & 0xFF;
    checksum -= packet[2 + i*2];
    checksum -= packet[3 + i*2];
  }
  packet[30] = checksum & 0xFF;
  packet[31] = (checksum >> 8) & 0xFF;
  Serial2.write(packet, 32);
}

void setup() {
  Serial2.begin(115200, SERIAL_8N1, 35, IBUS_TX); 
  pinMode(VOLTAGE_PIN, INPUT);
  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(76);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openReadingPipe(0, location);
  radio.startListening();
  
  dataToTX.rawDroneVoltage = analogRead(VOLTAGE_PIN);
  radio.writeAckPayload(0, &dataToTX, sizeof(dataToTX));
}

void loop() {

  if (radio.available()) {
    radio.read(&dataFromTX, sizeof(dataFromTX));
    lastRecvTime = millis();
    
    dataToTX.rawDroneVoltage = analogRead(VOLTAGE_PIN);
    radio.writeAckPayload(0, &dataToTX, sizeof(dataToTX));
    
    wasConnected = true;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastLoopTime >= loopInterval) {
    lastLoopTime = currentMillis;

    bool linkLost = (millis() - lastRecvTime > 500);
    if (linkLost) {
      channels[0]=1500; channels[1]=1500; channels[2]=1000; channels[3]=1500;
      channels[4]=1000; channels[5]=1000;

      if (wasConnected) {
        wasConnected = false;
        radio.flush_rx(); 
        radio.stopListening(); 
        delay(2); 
        radio.startListening();
      }
    } else {
      channels[0] = dataFromTX.roll;
      channels[1] = dataFromTX.pitch;
      channels[2] = dataFromTX.throttle; 
      channels[3] = dataFromTX.yaw; 
      channels[4] = dataFromTX.arm ? 2000 : 1000; 
      channels[5] = dataFromTX.aux2;
    }
    
    for (int i = 6; i < 14; i++) channels[i] = 1500;
    
    sendIBUS();
  }
}