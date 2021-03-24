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

const unsigned int tcpPort = 14552;
const unsigned int listenPort = 14551;
const unsigned int sendPort = 14550;

WiFiClient client;

size_t msgSize = 0;

mavlink_message_t currentUdpMsg, currentTcpMsg;
mavlink_status_t udpStatus, tcpStatus;

mavlink_message_t serialMsg;
mavlink_status_t serialStatusUDP;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(230400 * 2, SERIAL_8N1);
    //Serial.println("Starting connection");

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        //Serial.print(".");
    }

    //Serial.println("Connected");

    udp.begin(listenPort);
    sendUdp.begin(sendPort);

    client.setNoDelay(true);
    client.setTimeout(10);
}

void loop() {
    auto packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(incomingPacket, sizeof(incomingPacket));
        for (int i = 0; i < len; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, incomingPacket[i], &currentUdpMsg, &udpStatus)) {
                msgSize = mavlink_msg_to_send_buffer(serialBuffer, &currentUdpMsg);
                Serial.write(serialBuffer, msgSize);
                Serial.flush();
            }
        }
    }

//    while(client.available()){
//        char c = client.read();
//        if(mavlink_parse_char(MAVLINK_COMM_1, c, &currentTcpMsg, &tcpStatus)){
//            msgSize = mavlink_msg_to_send_buffer(serialBuffer, &currentTcpMsg);
//            Serial.write(serialBuffer, msgSize);
//            Serial.flush();
//        }
//    }

//    if(udp.remoteIP() and !client.connected()){
//        client.connect(udp.remoteIP(), tcpPort);
//        client.setNoDelay(true);
//    }

    //Recv from serial
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        if (mavlink_parse_char(MAVLINK_COMM_2, byte, &serialMsg, &serialStatusUDP)) {
            uint16_t msgLen = mavlink_msg_to_send_buffer(serialBuffer, &serialMsg);
            if (serialMsg.sysid == 2 and client.availableForWrite()) {
                client.write(serialBuffer, msgLen);
                client.flush(5);
            } else {
                sendUdp.beginPacket(udp.remoteIP(), sendPort);
                sendUdp.write(serialBuffer, msgLen);
                sendUdp.endPacket();
            }
        }
    }
}