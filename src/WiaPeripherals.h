
#ifndef __WIA_PERIPHERALS_H
#define __WIA_PERIPHERALS_H

#include "Arduino.h"
#include "Wire.h"

class WiaPeripherals{
  public:
  	WiaPeripherals();
  	byte get(void);
  	float temperature=0;
  	float humidity=0;

  private:
  	uint8_t _address;
};


#endif
