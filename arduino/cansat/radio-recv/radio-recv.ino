#include <SoftwareSerial.h>

// https://forum.arduino.cc/t/sending-data-with-apc220/627625/4

SoftwareSerial apc(4,7);

void setup()
{
   Serial.begin(115200);
   apc.begin(2400);
}

void loop()
{
  unsigned long now = millis();
  static unsigned long timer = 0;
  unsigned long interval = 100;
  if(now - timer >= interval)
  {
   timer = millis();
   Serial.print("Sending  ");
   Serial.println(now);
   apc.println(now);
  }
  if(apc.available() > 0)
  {
   Serial.print(char(apc.read()));
  }
}
