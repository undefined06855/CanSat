/*
 * All this does and needs to do really is get the shit from the radio and shove it back out the serial port at 115200 baud
 * We can (and should) send binary data
 */

#include <SoftwareSerial.h>

SoftwareSerial apc(9, 8); // tx, rx
uint8_t* buffer;
uint32_t right;

void setup() {
    Serial.begin(115200);
    apc.begin(2400);

    buffer = (uint8_t*)malloc(1024);
    right = 0;

    pinMode(13, OUTPUT);
}

void loop() {
    bool output = false;
    while (apc.available() > 0) {
        output = true;
        uint8_t one = apc.read();
        if (one == '\n') {
            // newline, send out buffer
            buffer[right] = 0x00;
            Serial.println((char*)buffer);
            right = 0;
            continue;
        }

        if (one == 0x00) {
            // uh
            one = '!';
        }

        buffer[right++] = one;
    }

    if (output) digitalWrite(13, HIGH);
    else digitalWrite(13, LOW);
}
