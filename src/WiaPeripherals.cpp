/*!
 *  @file DHT.cpp
 *
 *  @mainpage DHT series of low cost temperature/humidity sensors.
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for DHT series of low cost temperature/humidity sensors.
 *
 *  You must have Adafruit Unified Sensor Library library installed to use this
 * class.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Written by Adafruit Industries.
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */

#include "WiaPeripherals.h"
#include <Wire.h>

#define MIN_INTERVAL 2000 /**< min interval value */
#define TIMEOUT -1        /**< timeout on */

/*!
 *  @brief  Instantiates a new DHT class
 *  @param  pin
 *          pin number that sensor is connected
 *  @param  type
 *          type of sensor
 *  @param  count
 *          number of sensors
 */

WiaPeripherals::WiaPeripherals() {
}

/*!
 *  @brief  Setup sensor pins and set pull timings
 *  @param  usec
 *          Optionally pass pull-up time (in microseconds) before DHT reading
 *starts. Default is 55 (see function declaration in DHT.h).
 */
void WiaPeripherals::initTemperatureHumidity(uint8_t pin, uint8_t type, uint8_t count) {
  pullTime = 55;
  _pin = pin;
  _type = type;
  // set up the pins!
  
  pinMode(_pin, INPUT_PULLUP);
  // Using this value makes sure that millis() - lastreadtime will be
  // >= MIN_INTERVAL right away. Note that this assignment wraps around,
  // but so will the subtraction.
  _lastreadtime = millis() - MIN_INTERVAL;
  DEBUG_PRINT("DHT max clock cycles: ");
  DEBUG_PRINTLN(_maxcycles, DEC);
#ifdef __AVR
  _bit = digitalPinToBitMask(pin);
  _port = digitalPinToPort(pin);
#endif
  _maxcycles =
      microsecondsToClockCycles(1000); // 1 millisecond timeout for
                                       // reading pulses from DHT sensor.
  // Note that count is now ignored as the DHT reading algorithm adjusts itself
  // based on the speed of the processor.

}

/*!
 *  @brief  Read temperature
 *  @param  S
 *          Scale. Boolean value:
 *					- true = Fahrenheit
 *					- false = Celcius
 *  @param  force
 *          true if in force mode
 *	@return Temperature value in selected scale
 */
float WiaPeripherals::getTemperatureCelsius(bool force) {
  float f = NAN;

  if (read(force)) {
    switch (_type) {
    case DHT11:
      f = data[2];
      if (data[3] & 0x80) {
        f = -1 - f;
      }
      f += (data[3] & 0x0f) * 0.1;
      break;
    case DHT12:
      f = data[2];
      f += (data[3] & 0x0f) * 0.1;
      if (data[2] & 0x80) {
        f *= -1;
      }
      break;
    case DHT22:
    case DHT21:
      f = ((word)(data[2] & 0x7F)) << 8 | data[3];
      f *= 0.1;
      if (data[2] & 0x80) {
        f *= -1;
      }
      break;
    }
  }
  return f;
}



/*!
 *  @brief  Converts Celcius to Fahrenheit
 *  @param  c
 *					value in Celcius
 *	@return float value in Fahrenheit
 */
float WiaPeripherals::convertCtoF(float c) { return c * 1.8 + 32; }

/*!
 *  @brief  Converts Fahrenheit to Celcius
 *  @param  f
 *					value in Fahrenheit
 *	@return float value in Celcius
 */
float WiaPeripherals::convertFtoC(float f) { return (f - 32) * 0.55555; }

/*!
 *  @brief  Read Humidity
 *  @param  force
 *					force read mode
 *	@return float value - humidity in percent
 */
float WiaPeripherals::getHumidity(bool force) {
  float f = NAN;
  if (read(force)) {
    switch (_type) {
    case DHT11:
    case DHT12:
      f = data[0] + data[1] * 0.1;
      break;
    case DHT22:
    case DHT21:
      f = ((word)data[0]) << 8 | data[1];
      f *= 0.1;
      break;
    }
  }
  return f;
}


/*!
 *  @brief  Read value from sensor or return last one from less than two
 *seconds.
 *  @param  force
 *          true if using force mode
 *	@return float value
 */
bool WiaPeripherals::read(bool force) {
  // Check if sensor was read less than two seconds ago and return early
  // to use last reading.
  uint32_t currenttime = millis();
  if (!force && ((currenttime - _lastreadtime) < MIN_INTERVAL)) {
    return _lastresult; // return last correct measurement
  }
  _lastreadtime = currenttime;

  // Reset 40 bits of received data to zero.
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

#if defined(ESP8266)
  yield(); // Handle WiFi / reset software watchdog
#endif

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  pinMode(_pin, INPUT_PULLUP);
  delay(1);

  // First set data line low for a period according to sensor type
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  switch (_type) {
  case DHT22:
  case DHT21:
    delayMicroseconds(1100); // data sheet says "at least 1ms"
    break;
  case DHT11:
  default:
    delay(20); // data sheet says at least 18ms, 20ms just to be safe
    break;
  }

  uint32_t cycles[80];
  {
    // End the start signal by setting data line high for 40 microseconds.
    pinMode(_pin, INPUT_PULLUP);

    // Delay a moment to let sensor pull data line low.
    delayMicroseconds(pullTime);

    // Now start reading the data line to get the value from the DHT sensor.

    // Turn off interrupts temporarily because the next sections
    // are timing critical and we don't want any interruptions.
    InterruptLock lock;

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == TIMEOUT) {
      DEBUG_PRINTLN(F("DHT timeout waiting for start signal low pulse."));
      _lastresult = false;
      return _lastresult;
    }
    if (expectPulse(HIGH) == TIMEOUT) {
      DEBUG_PRINTLN(F("DHT timeout waiting for start signal high pulse."));
      _lastresult = false;
      return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed
    // all the pulses are read into a array and then examined in a later step.
    for (int i = 0; i < 80; i += 2) {
      cycles[i] = expectPulse(LOW);
      cycles[i + 1] = expectPulse(HIGH);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i = 0; i < 40; ++i) {
    uint32_t lowCycles = cycles[2 * i];
    uint32_t highCycles = cycles[2 * i + 1];
    if ((lowCycles == TIMEOUT) || (highCycles == TIMEOUT)) {
      DEBUG_PRINTLN(F("DHT timeout waiting for pulse."));
      _lastresult = false;
      return _lastresult;
    }
    data[i / 8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i / 8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  DEBUG_PRINTLN(F("Received from DHT:"));
  DEBUG_PRINT(data[0], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[1], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[2], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[3], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[4], HEX);
  DEBUG_PRINT(F(" =? "));
  DEBUG_PRINTLN((data[0] + data[1] + data[2] + data[3]) & 0xFF, HEX);

  // Check we read 40 bits and that the checksum matches.
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    _lastresult = true;
    return _lastresult;
  } else {
    DEBUG_PRINTLN(F("DHT checksum failure!"));
    _lastresult = false;
    return _lastresult;
  }
}

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
uint32_t WiaPeripherals::expectPulse(bool level) {
#if (F_CPU > 16000000L)
  uint32_t count = 0;
#else
  uint16_t count = 0; // To work fast enough on slower AVR boards
#endif
// On AVR platforms use direct GPIO port access as it's much faster and better
// for catching pulses that are 10's of microseconds in length:
#ifdef __AVR
  uint8_t portState = level ? _bit : 0;
  while ((*portInputRegister(_port) & _bit) == portState) {
    if (count++ >= _maxcycles) {
      return TIMEOUT; // Exceeded timeout, fail.
    }
  }
// Otherwise fall back to using digitalRead (this seems to be necessary on
// ESP8266 right now, perhaps bugs in direct port access functions?).
#else
  while (digitalRead(_pin) == level) {
    if (count++ >= _maxcycles) {
      return TIMEOUT; // Exceeded timeout, fail.
    }
  }
#endif

  return count;
=======
bool WiaPeripherals::init(int i2c_addr)
{
  uint8_t retry = 0;
  uint8_t chip_id = 0;


  _devAddr = i2c_addr;  
  Wire.begin(22,21);

  while((retry++ < 5) && (chip_id != 0x60))
  {
    chip_id = BME280Read8(BME280_REG_CHIPID);
#ifdef BMP280_DEBUG_PRINT
    Serial.print("Read chip ID: ");
    Serial.println(chip_id);
#endif
    delay(100);
  }

  dig_T1 = BME280Read16LE(BME280_REG_DIG_T1);
  dig_T2 = BME280ReadS16LE(BME280_REG_DIG_T2);
  dig_T3 = BME280ReadS16LE(BME280_REG_DIG_T3);

  dig_P1 = BME280Read16LE(BME280_REG_DIG_P1);
  dig_P2 = BME280ReadS16LE(BME280_REG_DIG_P2);
  dig_P3 = BME280ReadS16LE(BME280_REG_DIG_P3);
  dig_P4 = BME280ReadS16LE(BME280_REG_DIG_P4);
  dig_P5 = BME280ReadS16LE(BME280_REG_DIG_P5);
  dig_P6 = BME280ReadS16LE(BME280_REG_DIG_P6);
  dig_P7 = BME280ReadS16LE(BME280_REG_DIG_P7);
  dig_P8 = BME280ReadS16LE(BME280_REG_DIG_P8);
  dig_P9 = BME280ReadS16LE(BME280_REG_DIG_P9);

  dig_H1 = BME280Read8(BME280_REG_DIG_H1);
  dig_H2 = BME280Read16LE(BME280_REG_DIG_H2);
  dig_H3 = BME280Read8(BME280_REG_DIG_H3);
  dig_H4 = (BME280Read8(BME280_REG_DIG_H4) << 4) | (0x0F & BME280Read8(BME280_REG_DIG_H4 + 1));
  dig_H5 = (BME280Read8(BME280_REG_DIG_H5 + 1) << 4) | (0x0F & BME280Read8(BME280_REG_DIG_H5) >> 4);
  dig_H6 = (int8_t)BME280Read8(BME280_REG_DIG_H6);

  writeRegister(BME280_REG_CONTROLHUMID, 0x05);  //Choose 16X oversampling
  writeRegister(BME280_REG_CONTROL, 0xB7);  //Choose 16X oversampling

  return true;
}

float WiaPeripherals::getTemperatureCelsius(void)
{
  int32_t var1, var2;

  int32_t adc_T = BME280Read24(BME280_REG_TEMPDATA);
  // Check if the last transport successed
  if(!isTransport_OK) {
    return 0;
  }
  adc_T >>= 4;
  var1 = (((adc_T >> 3) - ((int32_t)(dig_T1 << 1))) *
    ((int32_t)dig_T2)) >> 11;

  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
    ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
    ((int32_t)dig_T3)) >> 14;

  t_fine = var1 + var2;
  float T = (t_fine * 5 + 128) >> 8;
  return T/100;
}


float WiaPeripherals::getTemperatureFahrenheit(void)
{
  int32_t var1, var2;

  int32_t adc_T = BME280Read24(BME280_REG_TEMPDATA);
  // Check if the last transport successed
  if(!isTransport_OK) {
    return 0;
  }
  adc_T >>= 4;
  var1 = (((adc_T >> 3) - ((int32_t)(dig_T1 << 1))) *
    ((int32_t)dig_T2)) >> 11;

  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
    ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
    ((int32_t)dig_T3)) >> 14;

  t_fine = var1 + var2;
  float T = (t_fine * 5 + 128) >> 8;
  return (((T/100) * 1.8) + 32);
}

uint32_t WiaPeripherals::getPressure(void)
{
  int64_t var1, var2, p;

  // Call getTemperatureCelsius to get t_fine
  getTemperatureCelsius();
  // Check if the last transport successed
  if(!isTransport_OK) {
    return 0;
  }

  int32_t adc_P = BME280Read24(BME280_REG_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
  var2 = var2 + (((int64_t)dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576-adc_P;
  p = (((p<<31)-var2)*3125)/var1;
  var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
  return (uint32_t)p/256;
}

uint32_t WiaPeripherals::getHumidity(void)
{
  int32_t v_x1_u32r, adc_H;

  // Call getTemperature to get t_fine
  getTemperatureCelsius();
  // Check if the last transport successed
  if(!isTransport_OK) {
    return 0;
  }

  adc_H = BME280Read16(BME280_REG_HUMIDITYDATA);

  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (uint32_t)(v_x1_u32r>>12)/1024.0;
}


uint8_t WiaPeripherals::BME280Read8(uint8_t reg)
{
  Wire.beginTransmission(_devAddr);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(_devAddr, 1);
  // return 0 if slave didn't response
  if(Wire.available() < 1) {
    isTransport_OK = false;
    return 0;
  } else {
    isTransport_OK = true;
  }

  return Wire.read();
}

uint16_t WiaPeripherals::BME280Read16(uint8_t reg)
{
  uint8_t msb, lsb;

  Wire.beginTransmission(_devAddr);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(_devAddr, 2);
  // return 0 if slave didn't response
  if(Wire.available() < 2) {
    isTransport_OK = false;
    return 0;
  } else {
    isTransport_OK = true;
  }
  msb = Wire.read();
  lsb = Wire.read();

  return (uint16_t) msb<<8 | lsb;
}

uint16_t WiaPeripherals::BME280Read16LE(uint8_t reg)
{
  uint16_t data = BME280Read16(reg);
  return (data >> 8) | (data << 8);
}

int16_t WiaPeripherals::BME280ReadS16(uint8_t reg)
{
  return (int16_t)BME280Read16(reg);
}

int16_t WiaPeripherals::BME280ReadS16LE(uint8_t reg)
{
  return (int16_t)BME280Read16LE(reg);
}

uint32_t WiaPeripherals::BME280Read24(uint8_t reg)
{
  uint32_t data;

  Wire.beginTransmission(_devAddr);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(_devAddr, 3);
  // return 0 if slave didn't response
  if(Wire.available() < 3) {
    isTransport_OK = false;
    return 0;
  }
  else if(isTransport_OK == false) {
    isTransport_OK = true;    
    if(!init(_devAddr)) {
#ifdef BMP280_DEBUG_PRINT      
      Serial.println("Device not connected or broken!");
#endif      
    }
  }
  data = Wire.read();
  data <<= 8;
  data |= Wire.read();
  data <<= 8;
  data |= Wire.read();

  return data;
}

void WiaPeripherals::writeRegister(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(_devAddr); // start transmission to device
  Wire.write(reg);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}
