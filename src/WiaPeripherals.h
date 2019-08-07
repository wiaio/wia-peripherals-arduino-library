/*!
 *  @file DHT.h
 *
 *  This is a library for DHT series of low cost temperature/humidity sensors.
 *
 *  You must have Adafruit Unified Sensor Library library installed to use this class.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Written by Adafruit Industries.
 *
 *  MIT license, all text above must be included in any redistribution
 */

#ifndef DHT_H
#define DHT_H

#include "Arduino.h"

/* Uncomment to enable printing out nice debug messages. */
//#define DHT_DEBUG


#define DEBUG_PRINTER Serial /**< Define where debug output will be printed. */

/* Setup debug printing macros. */
#ifdef DHT_DEBUG
  #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
  #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
  #define DEBUG_PRINT(...) {} /**< Debug Print Placeholder if Debug is disabled */
  #define DEBUG_PRINTLN(...) {} /**< Debug Print Line Placeholder if Debug is disabled */
#endif

/* Define types of sensors. */
#define DHT11 11 /**< DHT TYPE 11 */
#define DHT12 12 /**< DHY TYPE 12 */
#define DHT22 22 /**< DHT TYPE 22 */
#define DHT21 21 /**< DHT TYPE 21 */
#define AM2301 21 /**< AM2301 */

/*! 
 *  @brief  Class that stores state and functions for DHT
 */
class WiaPeripherals {
  public:
   WiaPeripherals();
   void initTemperatureHumidity(uint8_t pin, uint8_t type, uint8_t count=6);
   float getTemperatureCelsius(bool force=false);
   float convertCtoF(float);
   float convertFtoC(float);
   float getHumidity(bool force=false);
   bool read(bool force=false);

 private:
  uint8_t data[5];
  uint8_t _pin, _type;
  #ifdef __AVR
    // Use direct GPIO access on an 8-bit AVR so keep track of the port and bitmask
    // for the digital pin connected to the DHT.  Other platforms will use digitalRead.
    uint8_t _bit, _port;
  #endif
  uint32_t _lastreadtime, _maxcycles;
  bool _lastresult;
  uint8_t pullTime; // Time (in usec) to pull up data line before reading

  uint32_t expectPulse(bool level);

};

/*! 
 *  @brief  Class that defines Interrupt Lock Avaiability
 */
class InterruptLock {
  public:
   InterruptLock() {
#if !defined(ARDUINO_ARCH_NRF52)  
    noInterrupts();
#endif
   }
   ~InterruptLock() {
#if !defined(ARDUINO_ARCH_NRF52)  
    interrupts();
#endif
   }
=======
#pragma once

#ifndef _SEEED_BME280_H_
#define _SEEED_BME280_H_

#include <Arduino.h>
#include <Wire.h>

#define BME280_ADDRESS   0x76

#define BME280_REG_DIG_T1    0x88
#define BME280_REG_DIG_T2    0x8A
#define BME280_REG_DIG_T3    0x8C

#define BME280_REG_DIG_P1    0x8E
#define BME280_REG_DIG_P2    0x90
#define BME280_REG_DIG_P3    0x92
#define BME280_REG_DIG_P4    0x94
#define BME280_REG_DIG_P5    0x96
#define BME280_REG_DIG_P6    0x98
#define BME280_REG_DIG_P7    0x9A
#define BME280_REG_DIG_P8    0x9C
#define BME280_REG_DIG_P9    0x9E

#define BME280_REG_DIG_H1    0xA1
#define BME280_REG_DIG_H2    0xE1
#define BME280_REG_DIG_H3    0xE3
#define BME280_REG_DIG_H4    0xE4
#define BME280_REG_DIG_H5    0xE5
#define BME280_REG_DIG_H6    0xE7

#define BME280_REG_CHIPID          0xD0
#define BME280_REG_VERSION         0xD1
#define BME280_REG_SOFTRESET       0xE0

#define BME280_REG_CAL26           0xE1

#define BME280_REG_CONTROLHUMID    0xF2
#define BME280_REG_CONTROL         0xF4
#define BME280_REG_CONFIG          0xF5
#define BME280_REG_PRESSUREDATA    0xF7
#define BME280_REG_TEMPDATA        0xFA
#define BME280_REG_HUMIDITYDATA    0xFD

class WiaPeripherals
{
public:
  bool init(int i2c_addr = BME280_ADDRESS);
  float getTemperatureCelsius(void);
  float getTemperatureFahrenheit(void);
  uint32_t getPressure(void);
  uint32_t getHumidity(void);

private:
  int _devAddr;
  bool isTransport_OK;

  // Calibratino data
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t  dig_H6;
  int32_t t_fine;

  // private functions
  uint8_t BME280Read8(uint8_t reg);
  uint16_t BME280Read16(uint8_t reg);
  uint16_t BME280Read16LE(uint8_t reg);
  int16_t BME280ReadS16(uint8_t reg);
  int16_t BME280ReadS16LE(uint8_t reg);
  uint32_t BME280Read24(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
};

#endif
