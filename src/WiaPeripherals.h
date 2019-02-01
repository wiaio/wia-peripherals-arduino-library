
#ifndef __WIA_PERIPHERALS_H
#define __WIA_PERIPHERALS_H

#include "Arduino.h"
#include "Wire.h"

class WiaPeripherals{
  public:
  	WiaPeripherals();
    float getTemperature();
    float getHumidity();
  	float temperature=0;
  	float humidity=0;

  private:
    byte getTemperatureHumidity();
  	uint8_t _address;
};


#endif
