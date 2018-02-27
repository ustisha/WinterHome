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
#include <Controller.h>

const uint8_t OLED_CS = 8;
const uint8_t OLED_DC = 6;
const uint8_t OLED_RESET = 5;

const uint8_t SRV = 3;

const uint8_t DHT22 = 7;

class RemoteController : public Controller {
protected:
    DHT *dht;
    bool dhtIsError = false;

    uint8_t r1ThresholdAddress;
    float r1Threshold = 0.5;
    uint8_t r2ThresholdAddress;
    float r2Threshold = 1;
    uint8_t requiredTempAddress;
    float requiredTemp = 5;

    RotaryEncoder *encoder;
    uint16_t prevPosition = 0;
    uint8_t angleAddress;
    uint8_t displayState = STATE_INIT;

    Servo *srv;

    void displayInfo() {
        char delimiter[] = ": ";
        char text[33]{};
        strcat(text, "температура");
        strcat(text, delimiter);
        Format::temperature(text, requiredTemp);
        oled->drawUTF8(4, 16, text);

        char rText[20]{};
        strcat(rText, "реле 1");
        strcat(rText, delimiter);
        Format::temperature(rText, r1Threshold);
        oled->drawUTF8(4, 34, rText);

        rText[0] = 0;
        strcat(rText, "реле 2");
        strcat(rText, delimiter);
        Format::temperature(rText, r2Threshold);
        oled->drawUTF8(4, 52, rText);
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
            oled->drawUTF8(50, 24, "реле 1");
        } else if (relayPin == R2) {
            oled->drawUTF8(50, 24, "реле 2");
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

    bool relayIsOn(uint8_t pin) override {
        return !digitalRead(pin);
    }

    void relayOn(uint8_t pin) {
        bool change = !relayIsOn(pin);
        digitalWrite(pin, LOW);
        if (change) {
            render();
        }
    }

    void relayOff(uint8_t pin) {
        bool change = relayIsOn(pin);
        digitalWrite(pin, HIGH);
        if (change) {
            render();
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

    RemoteController(uint8_t cs, uint8_t dc, uint8_t reset) : Controller(cs, dc, reset) {
        pinMode(R1, OUTPUT);
        pinMode(R2, OUTPUT);
        digitalWrite(R1, HIGH);
        digitalWrite(R2, HIGH);

        EEPROM.setMemPool(0, EEPROMSizeNano);
        EEPROM.isReady();

        requiredTempAddress = (uint8_t) EEPROM.getAddress(sizeof(float));
        r1ThresholdAddress = (uint8_t) EEPROM.getAddress(sizeof(float));
        r2ThresholdAddress = (uint8_t) EEPROM.getAddress(sizeof(float));
        angleAddress = (uint8_t) EEPROM.getAddress(sizeof(long));

        requiredTemp = EEPROM.readFloat(requiredTempAddress);
        r1Threshold = EEPROM.readFloat(r1ThresholdAddress);
        r2Threshold = EEPROM.readFloat(r2ThresholdAddress);
        angle = EEPROM.readByte(angleAddress);

        srv = new Servo();
        srv->attach(SRV, 600, 3000);
        updateSrv(0);

        dht = new DHT();
        encoder = new RotaryEncoder(A2, A3);
    }

    void updateSrv(int16_t diff) {
        angle += diff * 10;
        if (angle < 0) {
            angle = 0;
        }
        if (angle > 180) {
            angle = 180;
        }
        srv->write(angle);
        EEPROM.updateLong(angleAddress, angle);
        render();
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
        render();
    }

    void render() override {
        oled->clearBuffer();

        oled->drawRFrame(0, 0, 128, 64, 4);
        oled->setFont(u8g2_font_mercutio_basic_nbp_t_all);

        if (displayState == STATE_INIT) {
            displayInfo();
        } else if (displayState == STATE_DISPLAY) {
            display();
        } else if (displayState == STATE_SET_TEMP) {
            displayTemp();
        } else if (displayState == STATE_SET_R1) {
            displayRelay(R1);
        } else if (displayState == STATE_SET_R2) {
            displayRelay(R2);
        }

        oled->setFont(u8g2_font_logisoso16_tf);

        if (displayState == STATE_DISPLAY) {
            if (dhtIsError) {
                oled->drawUTF8(35, 32, "ERROR!");
            } else {
                display16();
            }
        } else if (displayState == STATE_SET_TEMP) {
            displayTemp16();
        } else if (displayState == STATE_SET_R1) {
            displayRelay16(R1);
        } else if (displayState == STATE_SET_R2) {
            displayRelay16(R2);
        }

        oled->sendBuffer();
    }

    void setDisplayState(uint8_t state) {
        displayState = state;
        render();
    }

    uint8_t getDisplayState() {
        return displayState;
    }

    void sendData() {
        LoRa.beginPacket();

        LoRa.write(dhtIsError ? ERR_TEMP : 0);

        Float temp{};
        temp.f = currentTemp;
        LoRa.write(temp.b[0]);
        LoRa.write(temp.b[1]);
        LoRa.write(temp.b[2]);
        LoRa.write(temp.b[3]);

        Float hum{};
        hum.f = currentHum;
        LoRa.write(hum.b[0]);
        LoRa.write(hum.b[1]);
        LoRa.write(hum.b[2]);
        LoRa.write(hum.b[3]);

        LoRa.write(angle);
        LoRa.write((uint8_t) relayIsOn(R1));
        LoRa.write((uint8_t) relayIsOn(R2));

        LoRa.endPacket();
        LoRa.receive();
    }

    void tick() {
        tempControl();
        encoder->tick();
        uint16_t pos = (uint16_t) encoder->getPosition();
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

            uint8_t cmd = (uint8_t) LoRa.read();
            if (cmd == CMD_UP) {
                updateSrv(1);
                sendData();
            } else if (cmd == CMD_DOWN) {
                updateSrv(-1);
                sendData();
            }

            snr = LoRa.packetSnr();
        }
    }
};

RemoteController *ctrl;
Task *task;
Switcher *sw1;

void updateDHT22() {
    ctrl->updateDHT22();
}

void sendData() {
    ctrl->sendData();
}

void toDisplay() {
    ctrl->updateDHT22();
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
    task->each(sendData, 5000);
    task->one(toDisplay, 2000);

    sw1 = new Switcher(A7);
    sw1->addHandler(toSettings, Switcher::DEFAULT_PRESS);
}

void loop(void) {
    ctrl->tick();
    task->tick();
    sw1->tick();
}