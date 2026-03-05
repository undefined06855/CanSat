void setup() {
    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);
    Serial.begin(9600);
  }

  void loop() {
    Serial.print("SDA: ");
    Serial.print(digitalRead(A4));
    Serial.print("  SCL: ");
    Serial.println(digitalRead(A5));
    delay(1000);
  }