/// I made this because I am exhausted with OOP.
/// OOP is so abstract to me and I have a lot of other things to do :-(

#pragma once
#include <PCF8574.h>
#include <Adafruit_VL53L0X.h>
#define print Serial.print
#define println Serial.println
using ui8 = uint8_t;
using ui16 = uint16_t;


struct Simple_MultiVL53L0X{
  static Adafruit_VL53L0X sensorsArray[4];

  static void begin();
  static ui16 getDistance(ui8 id);

};