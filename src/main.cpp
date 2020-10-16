#include <Arduino.h>
#include <ESP8266WiFi.h>
//#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifiPassword.h"
#include <common/mavlink.h>

WiFiUDP udp, sendUdp;
uint8_t incomingPacket[8192];
uint8_t serialBuffer[8192];

const unsigned int listenPort = 14551;
const unsigned int sendPort = 14550;
mavlink_message_t serialMsg;
mavlink_message_t udpMsg;
mavlink_status_t status, serialStatus;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.println("Starting connection");

  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    //Serial.print(".");
  }

  //Serial.println("Connected");

  udp.begin(listenPort);
  sendUdp.begin(sendPort);
}

void loop() {
  //Recv from udp
  auto packetSize = udp.parsePacket();
  if(packetSize){
    int len = udp.read(incomingPacket, sizeof(incomingPacket));
    for(int i = 0; i < len; ++i){
      if(mavlink_parse_char(MAVLINK_COMM_0, incomingPacket[i], &udpMsg, &status)){
        //Recv msg from UDP, send it to serial
        uint16_t msgLen = mavlink_msg_to_send_buffer(serialBuffer, &udpMsg);
        Serial.write(serialBuffer, msgLen);
      }
    }
  }

  // // //Recv from serial
  while(Serial.available() > 0){
    uint8_t byte = Serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &serialMsg, &serialStatus)){

        //Send to udp
        uint16_t msgLen = mavlink_msg_to_send_buffer(serialBuffer, &serialMsg);
        sendUdp.beginPacket(udp.remoteIP(), sendPort);
        sendUdp.write(serialBuffer, msgLen);
        sendUdp.endPacket();
    }
  }

  //delay(10);
}