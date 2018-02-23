#include "Arduino.h"

#include <SPI.h>
#include <LoRa.h>
#include <U8g2lib.h>
#include <Servo.h>
#include <Format.h>
#include <DHT.h>
#include <Task.h>
#include <RotaryEncoder.h>
#include <Switcher.h>
#include <EEPROMex.h>
#include <EEPROMvar.h>

const uint8_t OLED_CS = 8;
const uint8_t OLED_DC = 6;
const uint8_t OLED_RESET = 5;

const uint8_t SRV = 3;

const uint8_t R1 = A0;
const uint8_t R2 = A1;

const uint8_t DHT22 = 7;

const uint8_t ENCODER1 = A2;
const uint8_t ENCODER2 = A3;
const uint8_t ENCODERSW = A7;

class Controller {
protected:
    U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI *oled;
    DHT *dht;
    bool dhtIsError = false;
    float currentTemp = 0;
    float currentHum = 0;

    int r1ThresholdAddress;
    float r1Threshold = 0.5;
    int r2ThresholdAddress;
    float r2Threshold = 1;
    int requiredTempAddress;
    float requiredTemp = 5;

    RotaryEncoder *encoder;
    long prevPosition = 0;
    int angleAddress;
    long angle = 0;
    uint8_t displayState = STATE_INIT;

    Servo *srv;

    char tempText[23]{};
    char r1Text[11]{};
    char r2Text[11]{};

    void displayInfo() {
        char delimiter[] = ": ";
        char text[33]{};
        strcat(text, tempText);
        strcat(text, delimiter);
        Format::temperature(text, requiredTemp);
        oled->drawUTF8(4, 16, text);

        char rText[20]{};
        strcat(rText, r1Text);
        strcat(rText, delimiter);
        Format::temperature(rText, r1Threshold);
        oled->drawUTF8(4, 34, rText);

        rText[0] = 0;
        strcat(rText, r2Text);
        strcat(rText, delimiter);
        Format::temperature(rText, r2Threshold);
        oled->drawUTF8(4, 52, rText);
    }

    void display() {
        oled->drawUTF8(16, 10, "темп.");
        oled->drawUTF8(60, 10, "влаж.");
        oled->drawUTF8(98, 10, "вент.");

        if (relayIsOn(R1)) {
            oled->drawBox(20, 42, 40, 18);
            oled->setDrawColor(2);
            oled->setFontMode(1);
        } else {
            oled->drawFrame(20, 42, 40, 18);
            oled->setDrawColor(1);
            oled->setFontMode(0);
        }
        oled->drawUTF8(24, 55, r1Text);

        if (relayIsOn(R2)) {
            oled->drawBox(64, 42, 40, 18);
            oled->setDrawColor(2);
            oled->setFontMode(1);
        } else {
            oled->drawFrame(64, 42, 40, 18);
            oled->setDrawColor(1);
            oled->setFontMode(0);
        }
        oled->drawUTF8(69, 55, r2Text);
    }

    void display16() {
        if (dhtIsError) {
            oled->drawUTF8(10, 32, "ERR!");
        } else {
            char tempOutput[10]{};
            Format::temperature(tempOutput, currentTemp);
            oled->drawUTF8(2, 32, tempOutput);
        }

        if (!dhtIsError) {
            char humOutput[6]{};
            Format::humidity(humOutput, currentHum);
            oled->drawUTF8(58, 32, humOutput);
        }

        char angleOutput[6]{};
        sprintf(angleOutput, "%ld%%", round(angle / 2));
        oled->drawUTF8(94, 32, angleOutput);
    }

    void displayTemp() {
        oled->drawUTF8(40, 10, "установка");
        oled->drawUTF8(34, 20, "температуры");
    }

    void displayTemp16() {
        char output[8]{};
        Format::temperature(output, requiredTemp, true);
        oled->drawUTF8(38, 47, output);
    }

    void displayRelay(uint8_t relayPin) {
        oled->drawUTF8(40, 10, "установка");
        if (relayPin == R1) {
            oled->drawUTF8(50, 24, r1Text);
        } else if (relayPin == R2) {
            oled->drawUTF8(50, 24, r2Text);
        }
    }

    void displayRelay16(uint8_t relayPin) {
        char tempOutput[10]{};
        if (relayPin == R1) {
            Format::temperature(tempOutput, r1Threshold);
        } else if (relayPin == R2) {
            Format::temperature(tempOutput, r2Threshold);
        }
        oled->drawUTF8(38, 47, tempOutput);
    }

    bool relayIsOn(uint8_t pin) {
        return !digitalRead(pin);
    }

    void relayOn(uint8_t pin) {
        bool change = !relayIsOn(pin);
        digitalWrite(pin, LOW);
        if (change) {
            this->render();
        }
    }

    void relayOff(uint8_t pin) {
        bool change = relayIsOn(pin);
        digitalWrite(pin, HIGH);
        if (change) {
            this->render();
        }
    }

    void tempControl() {
        if (dhtIsError) {
            relayOff(R1);
            relayOff(R2);
            return;
        }
        if (currentTemp <= (requiredTemp - r1Threshold)) {
            relayOn(R1);
            if (currentTemp <= (requiredTemp - r2Threshold)) {
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

    Controller(uint8_t cs, uint8_t dc, uint8_t reset) {
        strcpy(tempText, "температура");
        strcpy(r1Text, "реле 1");
        strcpy(r2Text, "реле 2");

        pinMode(R1, OUTPUT);
        pinMode(R2, OUTPUT);
        digitalWrite(R1, HIGH);
        digitalWrite(R2, HIGH);

        EEPROM.setMemPool(0, EEPROMSizeNano);
        EEPROM.isReady();

        requiredTempAddress = EEPROM.getAddress(sizeof(float));
        r1ThresholdAddress = EEPROM.getAddress(sizeof(float));
        r2ThresholdAddress = EEPROM.getAddress(sizeof(float));
        angleAddress = EEPROM.getAddress(sizeof(long));

        requiredTemp = EEPROM.readFloat(requiredTempAddress);
        r1Threshold = EEPROM.readFloat(r1ThresholdAddress);
        r2Threshold = EEPROM.readFloat(r2ThresholdAddress);
        angle = EEPROM.readLong(angleAddress);

        oled = new U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI(U8G2_R0, cs, dc, reset);
        oled->begin();
        render();

        srv = new Servo();
        srv->attach(SRV, 600, 3000);
        updateSrv();

        dht = new DHT();
        encoder = new RotaryEncoder(ENCODER1, ENCODER2);
    }

    void updateSrv() {
        srv->write(angle);
    }

    void updateDHT22() {
        int chk = dht->read22(DHT22);
        switch (chk) {
            case DHTLIB_OK:
                currentTemp = dht->temperature;
                currentHum = dht->humidity;
                break;
            case DHTLIB_ERROR_CHECKSUM:
            case DHTLIB_ERROR_TIMEOUT:
                dhtIsError = true;
                currentTemp = 0;
                currentHum = 0;
            default:
                break;
        }
        this->render();
    }

    void render() {
        oled->clearBuffer();

        oled->drawRFrame(0, 0, 128, 64, 4);
        oled->setFont(u8g2_font_mercutio_basic_nbp_t_all);

        if (this->displayState == STATE_INIT) {
            this->displayInfo();
        } else if (this->displayState == STATE_DISPLAY) {
            this->display();
        } else if (this->displayState == STATE_SET_TEMP) {
            this->displayTemp();
        } else if (this->displayState == STATE_SET_R1) {
            this->displayRelay(R1);
        } else if (this->displayState == STATE_SET_R2) {
            this->displayRelay(R2);
        }

        oled->setFont(u8g2_font_logisoso16_tf);

        if (this->displayState == STATE_DISPLAY) {
            this->display16();
        } else if (this->displayState == STATE_SET_TEMP) {
            this->displayTemp16();
        } else if (this->displayState == STATE_SET_R1) {
            this->displayRelay16(R1);
        } else if (this->displayState == STATE_SET_R2) {
            this->displayRelay16(R2);
        }

        oled->sendBuffer();
    }

    void setDisplayState(uint8_t state) {
        this->displayState = state;
        this->render();
    }

    uint8_t getDisplayState() {
        return this->displayState;
    }

    void tick() {
        this->tempControl();
        this->encoder->tick();
        long pos = encoder->getPosition();
        if (pos != prevPosition) {
            if (displayState == STATE_DISPLAY) {
                angle += (pos - prevPosition) * 5;
                if (angle < 0) {
                    angle = 0;
                }
                if (angle > 180) {
                    angle = 180;
                }
                updateSrv();
                EEPROM.updateLong(angleAddress, angle);
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
            this->render();
        }
    }
};

Controller *ctrl;
Task *task;
Switcher *sw1;

void updateDHT22() {
    ctrl->updateDHT22();
}

void toDisplay() {
    ctrl->updateDHT22();
    ctrl->setDisplayState(Controller::STATE_DISPLAY);
}

void toSettings() {
    if (ctrl->getDisplayState() == Controller::STATE_DISPLAY) {
        ctrl->setDisplayState(Controller::STATE_SET_TEMP);
    } else if (ctrl->getDisplayState() == Controller::STATE_SET_TEMP) {
        ctrl->setDisplayState(Controller::STATE_SET_R1);
    } else if (ctrl->getDisplayState() == Controller::STATE_SET_R1) {
        ctrl->setDisplayState(Controller::STATE_SET_R2);
    } else if (ctrl->getDisplayState() == Controller::STATE_SET_R2) {
        ctrl->setDisplayState(Controller::STATE_DISPLAY);
    }
}

void setup(void) {

    ctrl = new Controller(OLED_CS, OLED_DC, OLED_RESET);
    task = new Task();
    task->each(updateDHT22, 8000);
    task->one(toDisplay, 2000);

    sw1 = new Switcher(ENCODERSW);
    sw1->addHandler(toSettings, Switcher::DEFAULT_PRESS);
}

void loop(void) {
    ctrl->tick();
    task->tick();
    sw1->tick();
}