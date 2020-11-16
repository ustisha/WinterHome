#include "Arduino.h"

#include <SPI.h>
#include <LoRa.h>
#include <U8g2lib.h>
#include <Servo.h>
#include <Format.h>
#include <Wire.h>
#include <dht_nonblocking.h>
#include <Task.h>
#include <RotaryEncoder.h>
#include <Switcher.h>
#include <EEPROMex.h>
#include <EEPROMvar.h>
#include <Controller.h>

const uint8_t OLED_CS = 8;
const uint8_t OLED_DC = 6;
const uint8_t OLED_RESET = 5;

const uint8_t SRV = 3;
const uint8_t DHT22 = 4;

class RemoteController : public Controller {
protected:
    U8X8_SH1106_128X64_NONAME_4W_HW_SPI *oled;
    DHT_nonblocking *dht;
    Servo *srv;
    RotaryEncoder *encoder;

//    uint8_t relayMode = LOW;
    uint8_t relayMode = HIGH;
    bool tempReading = false;
    long prevPosition = 0;
    float r1Threshold = 0.5;
    float r2Threshold = 1;
    float requiredTemp = 5;
    uint8_t r1ThresholdAddress;
    uint8_t r2ThresholdAddress;
    uint8_t requiredTempAddress;
    uint8_t angleAddress;
    uint8_t displayState = STATE_INIT;

    void displayRelay(uint8_t relayPin) {
        char tempOutput[10]{};
        oled->drawUTF8(5, 0, "setup");
        if (relayPin == R1) {
            Format::temperature(tempOutput, r1Threshold, true);
            oled->drawUTF8(4, 1, "relay 1");
        } else if (relayPin == R2) {
            Format::temperature(tempOutput, r2Threshold, true);
            oled->drawUTF8(4, 1, "relay 2");
        }
        oled->drawUTF8(5, 3, tempOutput);
    }

    bool relayIsOn(uint8_t pin) override {
        if (relayMode) {
            return digitalRead(pin);
        }
        return !digitalRead(pin);
    }

    void relayOn(uint8_t pin) {
        bool change = !relayIsOn(pin);
        digitalWrite(pin, relayMode ? HIGH : LOW);
        if (change) {
            render();
        }
    }

    void relayOff(uint8_t pin) {
        bool change = relayIsOn(pin);
        digitalWrite(pin, relayMode ? LOW : HIGH);
        if (change) {
            render();
        }
    }

    void tempControl() {
        if (currentTemp.f <= (requiredTemp - r1Threshold)) {
            relayOn(R1);
            if (currentTemp.f <= (requiredTemp - r2Threshold)) {
                relayOn(R2);
            } else {
                relayOff(R2);
            }
        } else {
            relayOff(R1);
            relayOff(R2);
        }
    }

public:
    const static uint8_t STATE_INIT = 0;
    const static uint8_t STATE_DISPLAY = 1;
    const static uint8_t STATE_SET_TEMP = 2;
    const static uint8_t STATE_SET_R1 = 3;
    const static uint8_t STATE_SET_R2 = 4;

    RemoteController(uint8_t cs, uint8_t dc, uint8_t reset) : Controller(cs, dc, reset) {
        pinMode(R1, OUTPUT);
        pinMode(R2, OUTPUT);

        relayOff(R1);
        relayOff(R2);

        currentTemp.f = 0;
        currentHum.f = 0;

        oled = new U8X8_SH1106_128X64_NONAME_4W_HW_SPI(cs, dc, reset);
        oled->begin();

        EEPROM.setMemPool(0, EEPROMSizeNano);
        EEPROM.isReady();

        requiredTempAddress = (uint8_t) EEPROM.getAddress(sizeof(float));
        r1ThresholdAddress = (uint8_t) EEPROM.getAddress(sizeof(float));
        r2ThresholdAddress = (uint8_t) EEPROM.getAddress(sizeof(float));
        angleAddress = (uint8_t) EEPROM.getAddress(sizeof(long));

        requiredTemp = EEPROM.readFloat(requiredTempAddress);
        r1Threshold = EEPROM.readFloat(r1ThresholdAddress);
        r2Threshold = EEPROM.readFloat(r2ThresholdAddress);
        angle.i = EEPROM.readInt(angleAddress);

        srv = new Servo();
        srv->attach(SRV, 600, 3000);
        updateSrv(0);

        dht = new DHT_nonblocking(DHT22, DHT_TYPE_22);

        encoder = new RotaryEncoder(A2, A3);
    }

    void updateSrv(long diff) {
        angle.i += diff * 10;
        if (angle.i < 0) {
            angle.i = 0;
        }
        if (angle.i > 180) {
            angle.i = 180;
        }
        srv->write(angle.i);
        EEPROM.updateInt(angleAddress, angle.i);
        render();
    }

    void startReadingDHT22() {
        tempReading = true;
    }

    void render() override {
        oled->clearDisplay();
        oled->setFont(u8x8_font_pxplusibmcgathin_f);

        if (displayState == STATE_INIT) {
            char text[16]{};
            strcat(text, "   Temp: ");
            Format::temperature(text, requiredTemp);
            oled->drawUTF8(0, 0, text);

            text[0] = 0;
            strcat(text, "Relay 1: ");
            Format::temperature(text, r1Threshold);
            oled->drawUTF8(0, 2, text);

            text[0] = 0;
            strcat(text, "Relay 2: ");
            Format::temperature(text, r2Threshold);
            oled->drawUTF8(0, 4, text);
        } else if (displayState == STATE_DISPLAY) {
            if (relayIsOn(R1)) {
                oled->setInverseFont(true);
            }
            oled->drawUTF8(0, 0, "R1");
            oled->setInverseFont(false);

            if (relayIsOn(R2)) {
                oled->setInverseFont(true);
            }
            oled->drawUTF8(3, 0, "R2");
            oled->setInverseFont(false);

            char snrOutput[10]{};
            dtostrf(snr, 2, 0, snrOutput);
            strcat(snrOutput, "dB");
            oled->drawUTF8(oled->getCols() - 8, 0, snrOutput);

            float displayAngle = round((uint8_t) angle.i / 2);
            char angleString[6]{};
            char barOutput[16]{};
            for (int i = 0; i < 12; ++i) {
                if (i < displayAngle * 12 / 90) {
                    strcat(barOutput, "#");
                } else {
                    strcat(barOutput, " ");
                }
            }
            strcat(barOutput, ":");
            dtostrf(displayAngle, 2, 0, angleString);
            strcat(barOutput, angleString);
            strcat(barOutput, "%");
            oled->drawUTF8(0, 2, barOutput);

            char humOutput[18]{};
            strcpy(humOutput, "H:");
            Format::humidity(humOutput, currentHum.f);
            oled->drawUTF8(0, 4, humOutput);

            oled->setFont(u8x8_font_px437wyse700b_2x2_f);
            char tempOutput[18]{};
            strcpy(tempOutput, "T:");
            Format::temperature(tempOutput, currentTemp.f, true);
            oled->drawUTF8(0, 6, tempOutput);
        } else if (displayState == STATE_SET_TEMP) {
            oled->drawUTF8(5, 0, "setup");
            oled->drawUTF8(2, 1, "temperature");
            char output[8]{};
            Format::temperature(output, requiredTemp, true);
            oled->drawUTF8(5, 3, output);
        } else if (displayState == STATE_SET_R1) {
            displayRelay(R1);
        } else if (displayState == STATE_SET_R2) {
            displayRelay(R2);
        }
    }

    void setDisplayState(uint8_t state) {
        displayState = state;
        render();
    }

    uint8_t getDisplayState() {
        return displayState;
    }

    void sendData() {
        oled->drawUTF8(oled->getCols() - 2, 0, "\xBB");

        LoRa.beginPacket();

        LoRa.write(0);

        LoRa.write(currentTemp.b[0]);
        LoRa.write(currentTemp.b[1]);
        LoRa.write(currentTemp.b[2]);
        LoRa.write(currentTemp.b[3]);

        LoRa.write(currentHum.b[0]);
        LoRa.write(currentHum.b[1]);
        LoRa.write(currentHum.b[2]);
        LoRa.write(currentHum.b[3]);

        LoRa.write(0);
        LoRa.write(0);
        LoRa.write(0);
        LoRa.write(0);

        LoRa.write(angle.b[0]);
        LoRa.write(angle.b[1]);

        LoRa.write((uint8_t) relayIsOn(R1));
        LoRa.write((uint8_t) relayIsOn(R2));

        LoRa.endPacket();
        LoRa.receive();

        oled->drawUTF8(oled->getCols() - 1, 0, " ");
    }

    void tick() {
        if (tempReading && dht->measure(&currentTemp.f, &currentHum.f)) {
            tempReading = false;
            render();
            tempControl();
        }
        encoder->tick();
        long pos = encoder->getPosition();
        if (pos != prevPosition) {
            if (displayState == STATE_DISPLAY) {
                updateSrv(pos - prevPosition);
            } else if (displayState == STATE_SET_TEMP) {
                requiredTemp += ((pos - prevPosition) / 10.0);
                EEPROM.updateFloat(requiredTempAddress, requiredTemp);
            } else if (displayState == STATE_SET_R1) {
                r1Threshold += ((pos - prevPosition) / 10.0);
                EEPROM.updateFloat(r1ThresholdAddress, r1Threshold);
            } else if (displayState == STATE_SET_R2) {
                r2Threshold += ((pos - prevPosition) / 10.0);
                EEPROM.updateFloat(r2ThresholdAddress, r2Threshold);
            }
            prevPosition = pos;
            render();
        }

        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            oled->drawUTF8(oled->getCols() - 1, 0, "\xAB");

            uint8_t cmd = (uint8_t) LoRa.read();
            if (cmd == CMD_UP) {
                updateSrv(1);
                sendData();
            } else if (cmd == CMD_DOWN) {
                updateSrv(-1);
                sendData();
            }

            snr = LoRa.packetSnr();
            oled->drawUTF8(oled->getCols() - 1, 0, " ");
        }
    }
};

RemoteController *ctrl;
Task *task;
Switcher *sw1;

void updateDHT22() {
    ctrl->startReadingDHT22();
}

void sendData() {
    ctrl->sendData();
}

void toDisplay() {
    ctrl->startReadingDHT22();
    ctrl->setDisplayState(RemoteController::STATE_DISPLAY);
}

void toSettings() {
    if (ctrl->getDisplayState() == RemoteController::STATE_DISPLAY) {
        ctrl->setDisplayState(RemoteController::STATE_SET_TEMP);
    } else if (ctrl->getDisplayState() == RemoteController::STATE_SET_TEMP) {
        ctrl->setDisplayState(RemoteController::STATE_SET_R1);
    } else if (ctrl->getDisplayState() == RemoteController::STATE_SET_R1) {
        ctrl->setDisplayState(RemoteController::STATE_SET_R2);
    } else if (ctrl->getDisplayState() == RemoteController::STATE_SET_R2) {
        ctrl->setDisplayState(RemoteController::STATE_DISPLAY);
    }
}

void setup(void) {
    ctrl = new RemoteController(OLED_CS, OLED_DC, OLED_RESET);
    ctrl->render();

    task = new Task();
    task->each(updateDHT22, 8000);
    task->each(sendData, 10000);
    task->one(toDisplay, 5000);

    sw1 = new Switcher(A7);
    sw1->addHandler(toSettings, Switcher::DEFAULT_PRESS);
}

void loop(void) {
    ctrl->tick();
    task->tick();
    sw1->tick();
}