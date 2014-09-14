void setup() {
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);
/*  
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);
  OCR2A = 180;
  OCR2B = 50;
 */
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(13, HIGH);
  digitalWrite(11, HIGH);
  delayMicroseconds(500); // Approximately 10% duty cycle @ 1KHz
  digitalWrite(13, LOW);
  digitalWrite(11, LOW);
  delayMicroseconds(1000 - 500);
}
