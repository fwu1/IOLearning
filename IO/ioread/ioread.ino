
#include <avr/pgmspace.h>
#include <pins_arduino.h>

#define pin_PWM 13
#define pin_read 22
#define pin_din0 30

//volatile uint8_t *P_RS, *P_WR, *P_RD, *P_CS;
//uint8_t B_RS, B_WR, B_RD, B_CS;

volatile uint8_t *port_read,*port_din;
uint8_t bit_read;

void setup() {

  port_read = portInputRegister(digitalPinToPort(pin_read));
  bit_read = digitalPinToBitMask(pin_read);
  
  port_din = portInputRegister(digitalPinToPort(pin_din0));
  
  
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_read, INPUT);
  analogWrite(pin_PWM,15);
  Serial.begin(9600);
  Serial.println(*port_read);
  Serial.println(bit_read);
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


void readData()
{
  unsigned long i = 0; // test value
  unsigned long stop_time; // in milliseconds
  // calculate stop time (current time + 1 second)
  stop_time = millis() + 1000;
  uint8_t pcount=0;
  uint8_t ncount=0;
  uint8_t din=0;
  while(timer0_millis < stop_time) {
    pcount=0;
    ncount=0;
    //uint8_t d = (*port_read)& bit_read;
    // wait for signal high
    while(((*port_read)& bit_read)==0)
      ncount++;
    din=*port_din;  
    // wait for signal low
    while(((*port_read)& bit_read)!=0)
        pcount++;
    i++;
  }
  Serial.print("din=");
  Serial.println(din);
  
  
  // report performance results:
  // number of loop iterations in one second
  Serial.print(i);
  Serial.println(" pulse in one second.");
  if(i>0) {
    Serial.println(pcount);
    Serial.println(ncount);
  }
  else 
    Serial.println("No signal found");
}

void loop() {
  //waveDetect();
  readData();
  while(1); // and stop here
}
