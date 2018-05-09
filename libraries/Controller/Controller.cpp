#include "Arduino.h"
#include "Controller.h"

Controller::Controller(uint8_t cs, uint8_t dc, uint8_t reset) {
    // Задержка перед инициализацией.
    delay(1000);

    LoRa.begin(433E6);
    LoRa.setTxPower(17);
    LoRa.setSpreadingFactor(12);
    LoRa.enableCrc();
    LoRa.idle();

    // Задержка после инициализации.
    delay(1000);
}