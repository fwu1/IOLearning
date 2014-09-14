#include <Wire.h>
#include <avr/pgmspace.h>

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

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  Serial.println("REBOOT");  
  
  byte data;
  byte len=rdSensorReg8_8(0x00,&data);
  Serial.print("len=");
  Serial.println(len);
  Serial.print("data1=");
  Serial.println(data,HEX);
  byte idx=0xA0;
  for(;idx<=0xBF;idx++) {
    len=rdSensorReg8_8(idx,&data);
    if(len==1) {
      Serial.print(idx,HEX);
      Serial.print("=");
      Serial.println(data,HEX);
    }
   }
  /*
  byte err=wrSensorReg8_8(0xff, 0x0);
  Serial.print("error of wr 0xFF =");
  Serial.println(err);
  */
}

void loop() {
  // put your main code here, to run repeatedly:

}
