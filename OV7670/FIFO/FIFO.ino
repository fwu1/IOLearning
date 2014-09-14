
#include <Wire.h>
#include <ArduCAM.h>

ArduCAM myCAM(OV7670,A2,A1,A0,A3);

void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  Serial.println("REBOOT");  
  
  
  byte data;
  byte len;
  byte idx=0x00;
  myCAM.wrSensorReg8_8(0,0x01);
  for(;idx<=0x1F;idx++) {
    len=myCAM.rdSensorReg8_8(idx,&data);
    if(len==1) {
      Serial.print(idx,HEX);
      Serial.print("=");
      Serial.println(data,HEX);
    }
   }
   
}

void loop() {
  // put your main code here, to run repeatedly:

}
