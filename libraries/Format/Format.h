#ifndef ARDUINOEXAMPLE_TEMPERATURE_H
#define ARDUINOEXAMPLE_TEMPERATURE_H


class Format {
public:
    static void temperature(char *formatted, float t);
    static void temperature(char *formatted, float t, bool c);
    static void humidity(char *formatted, float h);
};

#endif
