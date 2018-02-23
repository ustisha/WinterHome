#include "Arduino.h"
#include "Switcher.h"

Switcher::Switcher(uint8_t swPin) {
    pin = swPin;
    start = 0;
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);
}

int Switcher::getIndex() {
    for (uint8_t i = 0; i < MAX; i++) {
        if (arr[i].cb == NULL) {
            return i;
        }
    }
    return -1;
}

int Switcher::sortByPress(const void *elem1, const void *elem2) {
    return ((Callback *) elem1)->press < ((Callback *) elem2)->press ? 1 : -1;
}

void Switcher::addHandler(void (*cb)(), unsigned int pressTime) {
    int i = getIndex();
    if (i >= 0) {
        arr[i].cb = cb;
        arr[i].press = pressTime;
    }
    qsort(arr, MAX, sizeof(Callback), Switcher::sortByPress);
}

bool Switcher::isPressed() {
    if (pin >= A0) {
        return analogRead(pin) == 0;
    } else {
        return digitalRead(pin) == 0;
    }
}

void Switcher::tick() {
    unsigned long m = millis();
    if (isPressed() && start == 0) {
        start = m;
        return;
    }
    if (!isPressed() && start != 0) {
        for (uint8_t i = 0; i < MAX; i++) {
            if (arr[i].cb != NULL && (m - start) >= arr[i].press) {
                arr[i].cb();
                break;
            }
        }
        start = 0;
    }
}
