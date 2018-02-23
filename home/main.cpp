#include "Arduino.h"

#include <LoRa.h>

void setup() {
    Serial.begin(57600);
    while (!Serial);

    Serial.println("LoRa Receiver");

    if (!LoRa.begin(433E6)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }

    LoRa.setTxPower(17);
    LoRa.setSpreadingFactor(12);
    LoRa.enableCrc();
}

union Float {
    float f;
    uint8_t b[4];
};

void loop() {
    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // received a packet
        Serial.println("Received packet");

        Float temp{};
        temp.b[0] = (uint8_t) LoRa.read();
        temp.b[1] = (uint8_t) LoRa.read();
        temp.b[2] = (uint8_t) LoRa.read();
        temp.b[3] = (uint8_t) LoRa.read();

        Serial.print("Temp: ");
        Serial.println(temp.f);

        Float hum{};
        hum.b[0] = (uint8_t) LoRa.read();
        hum.b[1] = (uint8_t) LoRa.read();
        hum.b[2] = (uint8_t) LoRa.read();
        hum.b[3] = (uint8_t) LoRa.read();

        Serial.print("Hum: ");
        Serial.println(hum.f);

        Serial.print("Angle: ");
        Serial.println(LoRa.read());

        Serial.print("R1: ");
        Serial.println(LoRa.read());

        Serial.print("R2: ");
        Serial.println(LoRa.read());

        // print RSSI of packet
        Serial.print("with RSSI ");
        Serial.print(LoRa.packetRssi());

        // print RSSI of packet
        Serial.print(" with SNR ");
        Serial.println(LoRa.packetSnr());

        Serial.println("------------------------");
    }
}
