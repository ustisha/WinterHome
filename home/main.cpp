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
    U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI *oled;

    uint8_t r1IsOn = 0;
    uint8_t r2IsOn = 0;

    uint8_t errCode = 0;

    unsigned long lastReceive = 0;
    unsigned long noSignal = 0;

public:
    HomeController(uint8_t cs, uint8_t dc, uint8_t reset) : Controller(cs, dc, reset) {
        oled = new U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI(U8G2_R0, cs, dc, reset);
        oled->begin();
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

        oled->setFont(u8g2_font_mercutio_basic_nbp_t_all);

        if (this->errCode == ERR_TEMP) {
            oled->drawUTF8(25, 20, "ошибка датчика");
            oled->drawUTF8(30, 40, "температуры!");
        } else if (this->noSignal != 0) {
            oled->drawUTF8(35, 35, "нет сигнала!");
            char noSignalOutput[10]{};
            sprintf(noSignalOutput, "%u мин.", (unsigned int) ceil(this->noSignal / 1000 / 60));
            oled->drawUTF8(50, 50, noSignalOutput);
        } else {
            char snrOutput[10]{};
            dtostrf(snr, 2, 0, snrOutput);
            strcat(snrOutput, "dB");
            oled->drawUTF8(80, 14, snrOutput);

            if (relayIsOn(R1)) {
                oled->drawBox(1, 1, 16, 16);
                oled->setDrawColor(2);
                oled->setFontMode(1);
            } else {
                oled->drawFrame(1, 1, 16, 16);
                oled->setDrawColor(1);
                oled->setFontMode(0);
            }
            oled->drawUTF8(4, 14, "Р1");

            if (relayIsOn(R2)) {
                oled->drawBox(20, 1, 16, 16);
                oled->setDrawColor(2);
                oled->setFontMode(1);
            } else {
                oled->drawFrame(20, 1, 16, 16);
                oled->setDrawColor(1);
                oled->setFontMode(0);
            }
            oled->drawUTF8(23, 14, "Р2");

            oled->drawFrame(1, 20, 126, 16);
            oled->setDrawColor(2);
            oled->setFontMode(1);

            float displayAngle = round((uint8_t) angle.i / 2);
            char angleString[18]{};
            char angleValueString[6]{};
            strcat(angleString, "вент.");
            dtostrf(displayAngle, 2, 0, angleValueString);
            strcat(angleString, angleValueString);
            strcat(angleString, "%");
            oled->drawUTF8(40, 33, angleString);

            long barLen = round(124 * displayAngle / 100);
            oled->drawBox(2, 21, barLen, 14);
            oled->setDrawColor(1);
            oled->setFontMode(0);

            char humOutput[16]{};
            strcat(humOutput, "влаж. ");
            Format::humidity(humOutput, currentHum.f);
            oled->drawUTF8(65, 49, humOutput);

            char pressOutput[24]{};
            strcat(pressOutput, "дав. ");
            Format::pressure(pressOutput, currentPressure.f, Format::PRESSURE_MMHG, false);
            oled->drawUTF8(65, 62, pressOutput);

            oled->setFont(u8g2_font_logisoso16_tf);

            char tempOutput[10]{};
            Format::temperature(tempOutput, currentTemp.f, true);
            oled->drawUTF8(2, 60, tempOutput);
        }

        oled->sendBuffer();
    }

    void upClick() {
        oled->drawUTF8(118, 14, "\xBB");
        oled->sendBuffer();

        LoRa.beginPacket();
        LoRa.write(CMD_UP);
        LoRa.endPacket();
        LoRa.receive();

        oled->drawUTF8(118, 14, " ");
        oled->sendBuffer();
    }

    void downClick() {
        oled->drawUTF8(118, 14, "\xBB");
        oled->sendBuffer();

        LoRa.beginPacket();
        LoRa.write(CMD_DOWN);
        LoRa.endPacket();
        LoRa.receive();

        oled->drawUTF8(118, 14, " ");
        oled->sendBuffer();
    }

    void tick() {
        unsigned long m = millis();

        if (lastReceive > m) {
            lastReceive = m;
        }

        if ((m - lastReceive) >= NO_SIGNAL_TIMEOUT) {
            noSignal = m - lastReceive;
        }

        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            oled->drawUTF8(118, 14, "\xAB");
            oled->sendBuffer();

            errCode = (uint8_t) LoRa.read();

            currentTemp.b[0] = (uint8_t) LoRa.read();
            currentTemp.b[1] = (uint8_t) LoRa.read();
            currentTemp.b[2] = (uint8_t) LoRa.read();
            currentTemp.b[3] = (uint8_t) LoRa.read();

            currentHum.b[0] = (uint8_t) LoRa.read();
            currentHum.b[1] = (uint8_t) LoRa.read();
            currentHum.b[2] = (uint8_t) LoRa.read();
            currentHum.b[3] = (uint8_t) LoRa.read();

            currentPressure.b[0] = (uint8_t) LoRa.read();
            currentPressure.b[1] = (uint8_t) LoRa.read();
            currentPressure.b[2] = (uint8_t) LoRa.read();
            currentPressure.b[3] = (uint8_t) LoRa.read();

            angle.b[0] = (uint8_t) LoRa.read();
            angle.b[1] = (uint8_t) LoRa.read();

            r1IsOn = (uint8_t) LoRa.read();
            r2IsOn = (uint8_t) LoRa.read();

            snr = LoRa.packetSnr();

            lastReceive = m;
            noSignal = 0;

            oled->drawUTF8(118, 14, " ");
            oled->sendBuffer();
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
