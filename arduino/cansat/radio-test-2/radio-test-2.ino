#include <SoftwareSerial.h>

// https://forum.arduino.cc/t/sending-data-with-apc220/627625/4

// sender
#define SNDR 1

SoftwareSerial apc(9, 8);

unsigned long timeSent = 0;
int received = 1;

void setup() {
    Serial.begin(9600);
    apc.begin(9600);
}

#ifdef SNDR
void loop() {
    if (received) {
        Serial.println("Sending...");
        apc.println("aaaaaaaaaaaaaa");
        timeSent = millis();
        received = 0;
    } else {
        if (millis() > timeSent + 2000) {
            Serial.println("Timed out!");
            received = 1;
        }
    }

   if (apc.available() > 0) {
        while (apc.available() > 0) {
            byte test = apc.read();
            Serial.print((char)test);
        }

        received = 1;

        Serial.println("received");
    }
}
#else
void loop() {
    // receiver code
    while (apc.available() > 0) {
        byte test = apc.read();
        apc.print((char)test);
        Serial.print((char)test);
    }
}
#endif
