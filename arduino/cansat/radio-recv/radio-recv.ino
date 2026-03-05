#include <SoftwareSerial.h>

// https://forum.arduino.cc/t/sending-data-with-apc220/627625/4

SoftwareSerial apc(9, 8);

void setup() {
   Serial.begin(9600);
   apc.begin(2400);
   digitalWrite(13, LOW);
}

unsigned long timeSent = 0;
int received = 1;

int timeSetHigh = 0;

void loop() {
  if (timeSetHigh + 200 < millis()) digitalWrite(13, LOW);
  /** // comment if this is for CANSAT
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
  //*/

  if (apc.available() > 0) {
    /** // comment if this is for CANSAT
    Serial.print("Ping: ");
    Serial.print((millis() - timeSent) / 2);
    Serial.println("ms");
    received = 1;
    //*/

    // uncomment if this is for GROUND
    apc.println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    digitalWrite(13, HIGH);
    timeSetHigh = millis();
    while (apc.available() > 0) {
      byte test = apc.read();
      Serial.print((char) test);
    }
  }
}
