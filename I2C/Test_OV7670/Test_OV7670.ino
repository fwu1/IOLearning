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
A0<-0
A1<-1
A2<-2
A3<-3
4<-4
5<-5
6<-6
7<-7
8<-PCLK
9<-HREF
10-> PWM to test
11-> (convert 5v to 3.3v) XCLK
12<-VSYNC
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
#define pin_read 12
#define pin_din0 4

volatile uint8_t *port_read,*port_din;
uint8_t bit_read;

extern volatile unsigned long timer0_millis;
unsigned long stop_time; // in milliseconds

#define pin_pclk 8
#define pin_href 9
#define pin_vsync 12
#define pin_din1 4
#define pin_din2 A0


volatile uint8_t *port_pclk,*port_href,*port_vsync,*port_din1,*port_din2;
uint8_t bit_pclk,bit_href,bit_vsync,bit_din1,bit_din2;
void getCamPorts()
{
  port_pclk = portInputRegister(digitalPinToPort(pin_pclk));
  bit_pclk = digitalPinToBitMask(pin_pclk);
  port_href = portInputRegister(digitalPinToPort(pin_href));
  bit_href = digitalPinToBitMask(pin_href);
  port_vsync = portInputRegister(digitalPinToPort(pin_vsync));
  bit_vsync = digitalPinToBitMask(pin_vsync);
  port_din1 = portInputRegister(digitalPinToPort(pin_din1));
  bit_din1 = digitalPinToBitMask(pin_din1);
  port_din2 = portInputRegister(digitalPinToPort(pin_din2));
  bit_din2 = digitalPinToBitMask(pin_din2);
  pinMode(pin_pclk, INPUT);
  pinMode(pin_href, INPUT);
  pinMode(pin_vsync, INPUT);
  pinMode(pin_din1, INPUT);
  pinMode(pin_din2, INPUT);
}


uint8_t buf[1300];
int vcount=0;
int hcount=0;
int dcount=0;
int sampleLine=221;

int readLineData()
{
    if(hcount!=sampleLine)
      return 0;
    uint8_t *b=buf;
    int cnt=0;
    byte dataState=0; //  0 - pclk is low, waiting next pclk high
                      //  1 - pclk is high, waiting for pclk to be low
    // wait to pclk to be low
    while(((*port_pclk)& bit_pclk!=0) && timer0_millis < stop_time);

    while((((*port_href)& bit_href)!=0) && timer0_millis < stop_time){
      byte pclk=(*port_pclk)& bit_pclk;
      if(dataState==0) {
         if(pclk!=0) {
           dataState=1;
           *b++=(PINC&15)|(PIND&240);
           cnt++;
           // read the data
         }
      }
      else if(dataState==1) {
         if(pclk==0) {
           dataState=0;
         }
      }
    }
  return cnt;  
}

void TestCamSignal()
{
  getCamPorts();
    // wait for signal high
  while(((*port_vsync)& bit_vsync)==0 && timer0_millis < stop_time);
  
  unsigned long start_time=millis();
  stop_time =  start_time+ 7000;
  
  while(timer0_millis < stop_time) {
    
    // wait for signal low
    while(((*port_vsync)& bit_vsync)!=0 && timer0_millis < stop_time);
    byte hrefState=0; //  0 - wait for href comming
                      //  1 - href is high, waiting for href to be low
                      //  2 - href is low, waiting next href high or vsync be high
    while(timer0_millis < stop_time){
      byte href=(*port_href)& bit_href;
      if(hrefState==0) {
         if(href!=0)
           hrefState=1;       
      }
      else if(hrefState==1) {
         if(href==0) {
           hrefState=2;
           if(vcount==0)
             hcount++;
         }
         else {
           int dsize=readLineData();
           if(hcount==sampleLine)
             dcount=dsize;
         }
      }
      else if(hrefState==2) {
        if(((*port_vsync)& bit_vsync)!=0)
          break;
         if(href!=0)
           hrefState=1;       
      }      
    }
    vcount++;
    if(vcount>0)
      break;
  }
  Serial.print("elapse time=");
  Serial.println(millis()-start_time);
  
  Serial.print("vcount=");
  Serial.println(vcount);
  Serial.print("hcount=");
  Serial.println(hcount);
  Serial.print("dcount=");
  Serial.println(dcount);
  
  for(int i=0;i<dcount;i++) {
    if(i%16==0)
      Serial.println();
    else
      Serial.print(" ");
    Serial.print(buf[i],HEX);
  }
  
  
}



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
  for(;idx<=0x3F;idx++) {
    len=rdSensorReg8_8(idx,&data);
    if(len==1) {
      Serial.print(idx,HEX);
      Serial.print("=");
      Serial.print(data,HEX);
      Serial.print(",");
      Serial.println(data);
    }
   }
}

#define wrReg wrSensorReg8_8
uint8_t rdReg(uint8_t reg) {
  uint8_t value;
  int len=rdSensorReg8_8(reg,&value);
  return value;
}

void setColor(uint8_t color){
	switch(color){
		case yuv422:
			wrSensorRegs8_8(yuv422_ov7670);
		break;
		case rgb565:
			wrSensorRegs8_8(rgb565_ov7670);
			{uint8_t temp=rdReg(0x11);
				delay(1);
				wrReg(0x11,temp);}//accorind to the linux kernel driver rgb565 PCLK needs re-writting
		break;
		case bayerRGB:
			wrSensorRegs8_8(bayerRGB_ov7670);
		break;
	}
}
void setRes(uint8_t res){
	switch(res){
		case vga:
			wrReg(REG_COM3,0);	// REG_COM3
			wrSensorRegs8_8(vga_ov7670);
		break;
		case qvga:
			wrReg(REG_COM3,4);	// REG_COM3 enable scaling
			wrSensorRegs8_8(qvga_ov7670);
		break;
		case qqvga:
			wrReg(REG_COM3,4);	// REG_COM3 enable scaling
			wrSensorRegs8_8(qqvga_ov7670);
		break;
	}
}

#define useVga
//#define useQvga
//#define useQqvga
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  setPin11ClkOut();
  camInit();
#ifdef useVga
	setRes(vga);
	setColor(bayerRGB);
	wrReg(0x11,25); // set freq of pclk
#elif defined(useQvga)
	setRes(qvga);
	setColor(yuv422);
	wrReg(0x11,12); // set freq of pclk
#else
	setRes(qqvga);
	setColor(yuv422);
	wrReg(0x11,3); // set freq of pclk
#endif
  // ref: http://wiki.jmoon.co/sensors/camera/ov7670/
  // change pclk 
  //wrReg(0x11,45); 
  
/*
  wrReg(REG_COM7,0x00);
  //wrReg(1,0x40);
  //wrReg(2,0x40);
  wrReg(0x13,0x00); //COM8 AGC control
*/

  bool myTest=true;
  if(myTest) {
    // Set gain
//    wrReg(1,0x40);
//    wrReg(2,0x40);
  
    #define YUV      0
    #define RGB      4
    #define BayerRaw  1
    #define ProcessedBayerRaw  5
    #define ColorBar  2
    
    // turn on/off color bar
    bool en_colorBar=true;
    //en_colorBar=false;
    if(en_colorBar) {
      wrReg(REG_COM7,ColorBar+BayerRaw);
      wrReg(0x70,0x80);
      wrReg(0x71,0x80);
      wrReg(0x11,45); 
    } else {
      wrReg(REG_COM7,BayerRaw);
      wrReg(0x70,0x00);
      wrReg(0x71,0x00);
      wrReg(0x11,45); 
      wrReg(0x3B,2); 
    }
    //control AGC AWB AEC
    
    #define EN_Fast_AGC_AEC  0x80
    #define EN_AGC  0x04
    #define EN_AWB  0x02
    #define EN_AEC  0x01
    
//    wrReg(REG_COM8,EN_AGC+EN_AEC);
    wrReg(REG_COM8,EN_Fast_AGC_AEC);
    wrReg(0x04,0x01);
    wrReg(0x10,0x00);
    wrReg(0xFF,0xFF);
  }
  readCamRegisters();
  
}

void loop() {
//  readData();
  TestCamSignal();
  while(1); 
  // put your main code here, to run repeatedly:

}
