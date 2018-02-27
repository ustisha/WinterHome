#include "Arduino.h"
#include "Controller.h"

Controller::Controller(uint8_t cs, uint8_t dc, uint8_t reset) {
    oled = new U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI(U8G2_R0, cs, dc, reset);
    oled->begin();

    LoRa.begin(433E6);
    LoRa.setTxPower(17);
    LoRa.setSpreadingFactor(12);
    LoRa.enableCrc();
}

void Controller::display16() {
    char tempOutput[10]{};
    Format::temperature(tempOutput, currentTemp);
    oled->drawUTF8(2, 32, tempOutput);

    char humOutput[6]{};
    Format::humidity(humOutput, currentHum);
    oled->drawUTF8(58, 32, humOutput);

    char angleOutput[6]{};
    sprintf(angleOutput, "%ld%%", round((uint8_t) angle / 2));
    oled->drawUTF8(94, 32, angleOutput);
}

void Controller::display() {
    oled->drawUTF8(16, 10, "темп.");
    oled->drawUTF8(60, 10, "влаж.");
    oled->drawUTF8(98, 10, "вент.");

    if (relayIsOn(R1)) {
        oled->drawBox(4, 42, 40, 18);
        oled->setDrawColor(2);
        oled->setFontMode(1);
    } else {
        oled->drawFrame(4, 42, 40, 18);
        oled->setDrawColor(1);
        oled->setFontMode(0);
    }
    oled->drawUTF8(8, 55, "реле 1");

    if (relayIsOn(R2)) {
        oled->drawBox(47, 42, 40, 18);
        oled->setDrawColor(2);
        oled->setFontMode(1);
    } else {
        oled->drawFrame(47, 42, 40, 18);
        oled->setDrawColor(1);
        oled->setFontMode(0);
    }
    oled->drawUTF8(52, 55, "реле 2");

    char snrOutput[4]{};
    dtostrf(snr, 2, 0, snrOutput);
    oled->drawUTF8(92, 45, "snr");
    oled->drawUTF8(102, 58, snrOutput);
}
