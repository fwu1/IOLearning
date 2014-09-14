
#include <avr/pgmspace.h>
#include <pins_arduino.h>

#define pin_PWM 13
#define pin_read 2

//volatile uint8_t *P_RS, *P_WR, *P_RD, *P_CS;
//uint8_t B_RS, B_WR, B_RD, B_CS;

volatile uint8_t *port_read;
uint8_t bit_read;

void setup() {
//  P_RS	= portOutputRegister(digitalPinToPort(pin_read));
//  B_RS	= digitalPinToBitMask(pin_read);

  port_read = portInputRegister(digitalPinToPort(pin_read));
  bit_read = digitalPinToBitMask(pin_read);
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_read, INPUT);
  analogWrite(pin_PWM,1);
  Serial.begin(9600);
  Serial.println("Started");
}
extern volatile unsigned long timer0_millis;

void waveDetect()
{
  unsigned long i = 0; // test value
  unsigned long stop_time; // in milliseconds
  // calculate stop time (current time + 1 second)
  stop_time = millis() + 1000;
  int last=0;
  unsigned long pcount=0;
  unsigned long ncount=0;
  while(timer0_millis < stop_time) {
    //int d=digitalRead(pin_read);
    uint8_t d = (*port_read)& bit_read;
    if (d==0)
        ncount++;
      else
        pcount++;
     
    if(d!=last) {
      i++;
      last=d;
    }
  }
  
  
  // report performance results:
  // number of loop iterations in one second
  Serial.print(i);
  Serial.println(" loops in one second.");
  if(i>0) {
    int p=pcount/i;
    int n=ncount/i;
    Serial.println(p);
    Serial.println(n);
    Serial.println(256*p/(p+n));
  }
  else 
    Serial.println("No signal found");
}

void loop() {
 waveDetect();
  while(1); // and stop here
}
