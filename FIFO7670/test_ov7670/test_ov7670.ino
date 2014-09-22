#include <Wire.h>
#include "OutputStream.h"
#include "CameraOV7670.h";
#include "CameraAL422B.h";


//CameraOV7670 c(Serial, 13,12);
CameraAL422B c(readData, 13,12,11,10);

unsigned char readData()
{
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
