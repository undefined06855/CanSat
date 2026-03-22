/*
 * Ground station passthrough: bridge binary data between APC220 and USB serial.
 */

#include <SoftwareSerial.h>

SoftwareSerial apc(9, 8); // tx, rx

void setup() {
    Serial.begin(115200);
    apc.begin(2400);

    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH); // keep APC220 in normal run mode

    pinMode(13, OUTPUT);
}

void loop() {
    bool output = false;
    while (apc.available() > 0) {
        output = true;
        Serial.write((uint8_t)apc.read());
    }

    if (output) digitalWrite(13, HIGH);
    else digitalWrite(13, LOW);

    while (Serial.available() > 0) {
        apc.write(Serial.read());
    }
}
