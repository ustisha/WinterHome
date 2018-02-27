#ifndef WINTERHOME_CONTROLLER_H
#define WINTERHOME_CONTROLLER_H

#include <Arduino.h>
#include <Format.h>
#include <LoRa.h>
#include <U8g2lib.h>

const uint8_t R1 = A0;
const uint8_t R2 = A1;

class Controller {

protected:
    static const uint8_t ERR_TEMP = 1;

    static const uint8_t CMD_UP = 1;
    static const uint8_t CMD_DOWN = 2;

    U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI *oled;
    float currentTemp = 0;
    float currentHum = 0;
    uint8_t angle = 0;
    float snr = 0;

    union Float {
        float f;
        uint8_t b[sizeof(float)];
    };

    void display();

    void display16();

    virtual bool relayIsOn(uint8_t pin)= 0;

public:

    Controller(uint8_t cs, uint8_t dc, uint8_t reset);

    virtual void render()= 0;
};

#endif //WINTERHOME_CONTROLLER_H
