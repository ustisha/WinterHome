#include "Arduino.h"
#include "Task.h"

Task::Task() = default;

int Task::getIndex() {
    for (int i = 0; i <= MAX; i++) {
        if (a[i].cb == NULL) {
            return i;
        }
    }
    return -1;
}

void Task::add(void (*cb)(), unsigned long t, uint8_t type) {
    int i = getIndex();
    if (i > -1) {
        a[i].cb = cb;
        a[i].type = type;
        a[i].last = millis();
        a[i].timeout = t;
    }
}

void Task::each(void (*cb)(), unsigned long t) {
    add(cb, t, TYPE_EACH);
}

void Task::one(void (*cb)(), unsigned long t) {
    add(cb, t, TYPE_ONE);
}

void Task::tick() {
    unsigned long m = millis();
    for (int i = 0; i <= MAX; i++) {
        if (a[i].cb != NULL && m >= (a[i].last + a[i].timeout)) {
            if (a[i].type == TYPE_EACH) {
                a[i].cb();
                a[i].last += a[i].timeout;
            } else if (a[i].type == TYPE_ONE) {
                a[i].cb();
                a[i].cb = NULL;
                a[i].last = NULL;
                a[i].timeout = NULL;
            }
        }
    }
}
