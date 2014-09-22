#include <Morse.h>

/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef MorseA_h
#define MorseA_h

#include "Arduino.h"

class MorseA
{
  public:
    MorseA(int pin);
    void dot();
    void dash();
  private:
    int _pin;
};

#endif
