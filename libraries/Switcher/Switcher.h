//
// Created by ustis on 16.02.2018.
//

#ifndef ARDUINOEXAMPLE_SWITCHER_H
#define ARDUINOEXAMPLE_SWITCHER_H

class Switcher {
    static const uint8_t MAX = 3;
protected:

    struct Callback {
        void (*cb)() = NULL;

        unsigned int press = 0;
    };

    Callback arr[Switcher::MAX];

    uint8_t pin;
    unsigned long start;

    static int sortByPress(const void *elem1, const void *elem2);

    int getIndex();

public:
    static const unsigned int DEFAULT_PRESS = 100;
    static const unsigned int DEFAULT_LONG = 1000;

    Switcher(uint8_t swPin);

    bool isPressed();

    void addHandler(void (*cb)(), unsigned int pressTime = DEFAULT_PRESS);

    void tick();
};


#endif //ARDUINOEXAMPLE_SWITCHER_H
