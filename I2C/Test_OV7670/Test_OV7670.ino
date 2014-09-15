#include <Wire.h>
#include <avr/pgmspace.h>
#include <pins_arduino.h>
#include "ov7670.h"
#include "ov7670_regs.h"

/*
The transfer rate is .5 mbps or 64kbytes per second
Connections
Order arduino ov7670
A5->SIOC
A4<->SIOD
11-> (convert 5v to 3.3v) XCLK
A0<-0
A1<-1
A2<-2
A3<-3
4<-4
5<-5
6<-6
7<-7
3<-VSYNC
2<-PCLK
*/


/* define a structure for sensor register initialization values */
struct sensor_reg {
       unsigned int reg;
       unsigned int val;
};

byte sensor_addr = 0x42;

byte wrSensorReg8_8(byte regID, byte regDat)
{

	//Serial.println("enter wrSensorReg");
	Wire.beginTransmission(sensor_addr>>1);
	Wire.write(regID); 	
	Wire.write(regDat); 	
	
	byte err=Wire.endTransmission();
  	return(err);
}

int wrSensorRegs8_8(const struct regval_list reglist[])
{
	int err = 0;
	unsigned int reg_addr,reg_val;
	const struct regval_list *next = reglist;
	
	while ((reg_addr != 0xff) | (reg_val != 0xff))
	{		
		reg_addr = pgm_read_word(&next->reg_num);
		reg_val = pgm_read_word(&next->value);
		err = wrSensorReg8_8(reg_addr, reg_val);
		if (!err)
			return err;
	   	next++;
	} 
	
	return 1;
}

/*
int wrSensorRegs8_8(const struct sensor_reg reglist[])
{
	int err = 0;
	unsigned int reg_addr,reg_val;
	const struct sensor_reg *next = reglist;
	
	while ((reg_addr != 0xff) | (reg_val != 0xff))
	{		
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		err = wrSensorReg8_8(reg_addr, reg_val);
		if (!err)
			return err;
	   	next++;
	} 
	
	return 1;
}

*/
byte rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
{

	//Serial.println("enter wrSensorReg");
	Wire.beginTransmission(sensor_addr >> 1);
	Wire.write(regID & 0x00FF); 	
	Wire.endTransmission();
	
	Wire.requestFrom((sensor_addr >> 1),1);
	if(Wire.available())
		*regDat = Wire.read(); 	

	delay(1);
  	return(1);
}


void setPin11ClkOut()
{
/* Setup the 8mhz PWM clock 
  This will be on pin 11*/
  DDRB|=(1<<3);//pin 11
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));
  TCCR2A=(1<<COM2A0)|(1<<WGM21)|(1<<WGM20);
  TCCR2B=(1<<WGM22)|(1<<CS20);
  OCR2A=0;//(F_CPU)/(2*(X+1))
  // The output clk frequence is 16M/(2*(OCR2A+1)
}

#define pin_PWM 10
#define pin_read 2
#define pin_din0 8


volatile uint8_t *port_read,*port_din;
uint8_t bit_read;

extern volatile unsigned long timer0_millis;


void readData()
{
  unsigned long i = 0; // test value
  unsigned long stop_time; // in milliseconds
  Serial.println("Read Data");  
  port_read = portInputRegister(digitalPinToPort(pin_read));
  bit_read = digitalPinToBitMask(pin_read);
  
  port_din = portInputRegister(digitalPinToPort(pin_din0));

  pinMode(pin_PWM, OUTPUT);
  analogWrite(pin_PWM,4);
  pinMode(pin_read, INPUT);

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
    while(((*port_read)& bit_read)==0 && timer0_millis < stop_time)
      ncount++;
    din=*port_din;  
    // wait for signal low
    while(((*port_read)& bit_read)!=0 && timer0_millis < stop_time)
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

void camInit(void){
	wrSensorReg8_8(0x12, 0x80);
	delay(100);
	wrSensorRegs8_8(ov7670_default_regs);
	wrSensorReg8_8(REG_COM10,32);//pclk does not toggle on HBLANK
}

void readCamRegisters()
{
  byte data;
  byte len=rdSensorReg8_8(0x00,&data);
  Serial.print("len=");
  Serial.println(len);
  Serial.print("data1=");
  Serial.println(data,HEX);
  byte idx=0x00;
  for(;idx<=0x8F;idx++) {
    len=rdSensorReg8_8(idx,&data);
    if(len==1) {
      Serial.print(idx,HEX);
      Serial.print("=");
      Serial.println(data,HEX);
    }
   }
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  setPin11ClkOut();
  //readCamRegisters  ();
  camInit();
  
}

void loop() {
  readData();
  while(1); 
  // put your main code here, to run repeatedly:

}
