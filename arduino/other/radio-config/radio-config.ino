//

/*
   1

   apc220.println("WR 433900 3 9 3 0");
   To configure the APC220 you need to set the SET pin HIGH and then pull it down (set it to LOW).
   This will put the module in configuration mode. Once it is in configuration mode you can write
   the configuration to it as displayed in the above example.
   The format is
   1

   WR Frequency RFDataRate OutputPower UART-Rate Series check
   Possible values for all these settings:

    Frequency: Unit is KHz,for example 434MHz is 434000
    RF Data Rate: 1,2,3 and 4 refer to 2400,4800,9600,19200bps
    Output Power: 0 to 9, 9 means 13dBm(20mW)
    UART Rate: 0,1,2,3,4,5 and 6 refers to 1200,2400,4800,9600,19200,38400,57600bps
    Series Checkout: Series checkout:0 means no check,1 means even parity,2 means odd parity.

*/

#include <SoftwareSerial.h>
/*
    1 SET connected to Arduino pin 13
    2 AUX not connected
    3 TXD connected to pin 4
    4 RXD connected to pin 7
    5 EN not connected
    6 VCC connected to 5V
    7 GND connected to ground
*/

const int pinRX = 4;
const int pinTX = 7;
const int pinSET = 13;

SoftwareSerial apc220(pinRX, pinTX); // Crt softserial port and bind tx / rx to appropriate PINS

void setupSoftAPC(void)
{
   pinMode(pinSET, HIGH);
   apc220.begin(9600);
}

void setSettings(void)
{
   digitalWrite(pinSET, LOW); // pulling SET to low will put apc220 in config mode
   delay(10); // stabilize please
   //  apc220.println("WR 433900 1 9 1 0");
   apc220.println("WR 433900 1 9 1 0"); // change ss baud rate
   delay(10);

   while (apc220.available())
   {
      Serial.write(apc220.read());
   }
   digitalWrite(pinSET, HIGH); // put apc220 back in operation
   delay(200);
}
void getSettings(void)
{
   digitalWrite(pinSET, LOW); // pulling SET to low will put apc220 in config mode
   delay(10); // stabilize please
   apc220.println("RD"); // ask for data
   delay(10);
   Serial.println("reading  ");
   while (apc220.available())
   {
      Serial.write(apc220.read());
   }
   digitalWrite(pinSET, HIGH); // put apc220 back in operation
   delay(200);
}

void setup()
{
   Serial.begin(9600);
   setupSoftAPC();
   setSettings();
   getSettings();
}

void loop()
{
   static unsigned long radioTimer = 0;
   unsigned long radioInterval = 5000;
   if (millis() - radioTimer >= radioInterval)
   {
      radioTimer = millis();
      apc220.println("Hello World!");
   }

   if (apc220.available())
   {
      Serial.println(char(apc220.read()));
   }
}
