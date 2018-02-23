
#ifndef ARDUINOEXAMPLE_TASK_H
#define ARDUINOEXAMPLE_TASK_H


class Task {
    const static uint8_t MAX = 5;
    const uint8_t TYPE_EACH = 1;
    const uint8_t TYPE_ONE = 2;

    struct Callback {
        void (*cb)() = NULL;

        uint8_t type = 0;
        unsigned long last = 0;
        unsigned long timeout = 0;
    };

    Callback a[MAX]{};
protected:
    int getIndex();

    void add(void (*cb)(), unsigned long t, uint8_t type);

public:
    Task();

    void each(void (*cb)(), unsigned long t);

    void one(void (*cb)(), unsigned long t);

    void tick();
};


#endif //ARDUINOEXAMPLE_TASK_H
