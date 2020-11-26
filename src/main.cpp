#include <Arduino.h>
#include <ESP8266WiFi.h>
//#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifiPassword.h"
#include <common/mavlink.h>
#include <queue>

WiFiUDP udp, sendUdp;
uint8_t incomingPacket[8192];
uint8_t serialBuffer[8192];

const unsigned int listenPort = 14551;
const unsigned int sendPort = 14550;
mavlink_message_t serialMsg;
std::queue<mavlink_message_t> udpMsg;


size_t currentByteSent = 0;
size_t msgSize = 0;

mavlink_message_t currentUdpMsg;
mavlink_status_t status, serialStatus;
volatile bool cts = false;

ICACHE_RAM_ATTR void picInterrupt(){
  cts = true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200, SERIAL_8N1);
  //Serial.println("Starting connection");

  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    //Serial.print(".");
  }

  //Serial.println("Connected");

  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), picInterrupt, CHANGE);

  udp.begin(listenPort);
  sendUdp.begin(sendPort);
}

void loop() {
  //Recv from udp, store in buffer
  if(udpMsg.size() < 5){
    auto packetSize = udp.parsePacket();
    if(packetSize){
      int len = udp.read(incomingPacket, sizeof(incomingPacket));
      for(int i = 0; i < len; ++i){
        if(mavlink_parse_char(MAVLINK_COMM_0, incomingPacket[i], &currentUdpMsg, &status)){
          udpMsg.emplace(currentUdpMsg);
          memset(&currentUdpMsg, 0, sizeof(currentUdpMsg));
          memset(&status, 0, sizeof(status));
        }
      }
    }
  }

  if(cts && !udpMsg.empty()){
    const auto &msg = udpMsg.front();
    if(currentByteSent == 0){
      msgSize = mavlink_msg_to_send_buffer(serialBuffer, &msg);
    }

    if(currentByteSent >= msgSize){
      udpMsg.pop();
      currentByteSent = 0;
    } else {
      Serial.write(serialBuffer[currentByteSent++]);
      cts = false;
    }
  }

  // // //Recv from serial
  if(Serial.available() > 0){
    uint8_t byte = Serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &serialMsg, &serialStatus)){

        //Send to udp
        uint16_t msgLen = mavlink_msg_to_send_buffer(serialBuffer, &serialMsg);
        sendUdp.beginPacket(udp.remoteIP(), sendPort);
        sendUdp.write(serialBuffer, msgLen);
        sendUdp.endPacket();
        return;
    }
  }
}