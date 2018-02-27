#include "Arduino.h"

#include <SPI.h>
#include <Switcher.h>
#include <Controller.h>
#include <Task.h>

const uint8_t OLED_CS = 8;
const uint8_t OLED_DC = 6;
const uint8_t OLED_RESET = 5;

class HomeController : public Controller {

    const uint16_t NO_SIGNAL_TIMEOUT = 60000;

protected:
    uint8_t r1IsOn = 0;
    uint8_t r2IsOn = 0;

    uint8_t errCode = 0;

    unsigned long lastReceive = 0;
    unsigned long noSignal = 0;

public:
    HomeController(uint8_t cs, uint8_t dc, uint8_t reset) : Controller(cs, dc, reset) {

    }

    bool relayIsOn(uint8_t pin) override {
        if (pin == R1) {
            return (bool) r1IsOn;
        }
        if (pin == R2) {
            return (bool) r2IsOn;
        }
        return false;
    }

    void render() override {
        oled->clearBuffer();

        oled->drawRFrame(0, 0, 128, 64, 4);
        oled->setFont(u8g2_font_mercutio_basic_nbp_t_all);

        if (this->errCode == ERR_TEMP) {
            oled->drawUTF8(25, 20, "ошибка датчика");
            oled->drawUTF8(30, 40, "температуры!");
        } else if (this->noSignal != 0) {
            oled->drawUTF8(35, 20, "нет сигнала!");
        } else {
            this->display();
        }

        oled->setFont(u8g2_font_logisoso16_tf);

        if (this->errCode != 0) {

        } else if (this->noSignal != 0) {
            char noSignalOutput[10]{};
            sprintf(noSignalOutput, "%u min.", (unsigned int) ceil(this->noSignal / 1000 / 60));
            oled->drawUTF8(38, 50, noSignalOutput);
        } else {
            this->display16();
        }

        oled->sendBuffer();
    }

    void upClick() {
        LoRa.beginPacket();
        LoRa.write(CMD_UP);
        LoRa.endPacket();
        LoRa.receive();
    }

    void downClick() {
        LoRa.beginPacket();
        LoRa.write(CMD_DOWN);
        LoRa.endPacket();
        LoRa.receive();
    }

    void tick() {
        unsigned long m = millis();

        if ((m - lastReceive) >= NO_SIGNAL_TIMEOUT) {
            noSignal = m - lastReceive;
        }

        int packetSize = LoRa.parsePacket();
        if (packetSize) {

            this->errCode = (uint8_t) LoRa.read();

            Float temp{};
            temp.b[0] = (uint8_t) LoRa.read();
            temp.b[1] = (uint8_t) LoRa.read();
            temp.b[2] = (uint8_t) LoRa.read();
            temp.b[3] = (uint8_t) LoRa.read();
            this->currentTemp = temp.f;

            Float hum{};
            hum.b[0] = (uint8_t) LoRa.read();
            hum.b[1] = (uint8_t) LoRa.read();
            hum.b[2] = (uint8_t) LoRa.read();
            hum.b[3] = (uint8_t) LoRa.read();
            this->currentHum = hum.f;

            this->angle = (uint8_t) LoRa.read();

            this->r1IsOn = (uint8_t) LoRa.read();
            this->r2IsOn = (uint8_t) LoRa.read();

            this->snr = LoRa.packetSnr();

            this->lastReceive = m;
            this->noSignal = 0;
        }
    }
};

HomeController *ctrl;
Task *task;
Switcher *swUp;
Switcher *swDown;

void renderDisplay() {
    ctrl->render();
}

void upClick() {
    ctrl->upClick();
}

void downClick() {
    ctrl->downClick();
}

void setup() {
    ctrl = new HomeController(OLED_CS, OLED_DC, OLED_RESET);
    ctrl->render();
    task = new Task();
    task->each(renderDisplay, 1000);

    swUp = new Switcher(A1);
    swUp->addHandler(upClick, Switcher::DEFAULT_PRESS);

    swDown = new Switcher(A0);
    swDown->addHandler(downClick, Switcher::DEFAULT_PRESS);
}

void loop() {
    ctrl->tick();
    task->tick();
    swUp->tick();
    swDown->tick();
}
