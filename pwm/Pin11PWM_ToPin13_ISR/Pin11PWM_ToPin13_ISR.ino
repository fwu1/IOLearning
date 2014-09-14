void setup() {
  bitSet(DDRB, 5); // LED pin
  bitSet(PCICR, PCIE0); // enable pin change interrupts on bank 0
  bitSet(PCMSK0, PCINT3); // enable PCINT3 (PB3) pin change interrupt
}

byte pwm = 0;

void loop() {
  analogWrite(11, pwm); // set the PWM duty cycle
  delay(10); // a very short delay
  pwm++; // increase the PWM duty cycle
}

ISR(PCINT0_vect) {
  // read the value of pin 11
  if(bitRead(PINB, 3)) {
    // set pin 13 high
    bitSet(PORTB, 5); // LED on
  } else {
    bitClear(PORTB, 5); // LED off
  }
}
