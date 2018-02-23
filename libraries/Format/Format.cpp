#include "Arduino.h"
#include "Format.h"

void Format::temperature(char *formatted, float tempInput) {
    Format::temperature(formatted, tempInput, false);
}

void Format::temperature(char *formatted, float tempInput, bool c) {
    char tempString[10]{};

    if (tempInput < 0) {
        strcat(formatted, "-");
    } else {
        strcat(formatted, " ");
    }
    dtostrf(abs(tempInput), 3, 1, tempString);
    strcat(formatted, tempString);
    if (c) {
        strcat(formatted, "C");
    }
    strcat(formatted, "°");
}

void Format::humidity(char *formatted, float h) {
    dtostrf(h, 2, 0, formatted);
    strcat(formatted, "%");
}
