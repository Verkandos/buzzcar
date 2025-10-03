#include "Screen.h"
#include <Wire.h>
#include <U8g2lib.h>
#include "GPIOManager.hpp"

// Single OLED instance scoped to this translation unit
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);


//sets up screen
void screenBegin(uint8_t i2cSDA, uint8_t i2cSCL) {
  // Use GPIOManager to configure I2C pins
  GPIOManager& gpio = GPIOManager::getInstance();
  gpio.configureI2C(i2cSDA, i2cSCL, 400000); // 400kHz I2C

  Serial.println("Screen Begin");


  // Wire.begin(i2cSDA, i2cSCL);
  u8g2.setBusClock(400000);
  u8g2.begin();
  u8g2.setPowerSave(0);
}

//handles the monitor display based on direction
// 0 = STOP 1 = FORWARD 2 = LEFT 3 = RIGHT
void showDirection(int direction) {
  Serial.println("Screen.cpp showing direction....");
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "TEST:");
  u8g2.drawStr(0, 28, "WORKING");

  const char* label = "UNKNOWN";
  switch (direction) {
    case 0: label = "STOP";    break;
    case 1: label = "FORWARD"; break;
    case 2: label = "LEFT";    break;
    case 3: label = "RIGHT";   break;
  }

  u8g2.drawStr(0, 48, label);
  u8g2.sendBuffer();
}
