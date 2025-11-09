#include "Screen.h"
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>
#include "GPIOManager.hpp"


static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

static void drawCentered(const char* s, int y) {
  u8g2_uint_t w = u8g2.getStrWidth(s);
  int x = (128 - (int)w) / 2;
  if (x < 0) x = 0;
  u8g2.drawStr(x, y, s);
}

// draw all arrow functions
static void drawArrowUp(int cx, int cy) {
  u8g2.drawTriangle(cx, cy-18, cx-12, cy-4, cx+12, cy-4);
  u8g2.drawBox(cx-4, cy-4, 8, 18);
}

static void drawArrowLeft(int cx, int cy) {
  u8g2.drawTriangle(cx-18, cy, cx-4, cy-12, cx-4, cy+12);
  u8g2.drawBox(cx-4, cy-3, 18, 6);
}

static void drawArrowRight(int cx, int cy) {
  u8g2.drawTriangle(cx+18, cy, cx+4, cy-12, cx+4, cy+12);
  u8g2.drawBox(cx-14, cy-3, 18, 6);
}

static void drawStopOctagonFilled(int cx, int cy, int r) {
  const float step = (float)M_PI / 4.0f; 
  const float offset = step / 2.0f;   
  int vx[8], vy[8];

  for (int i = 0; i < 8; i++) {
    float a = offset + step * i;
    vx[i] = cx + (int)roundf(r * cosf(a));
    vy[i] = cy + (int)roundf(r * sinf(a));
  }

  // Fill the octagon
  for (int i = 0; i < 8; i++) {
    int j = (i + 1) % 8;
    u8g2.drawTriangle(cx, cy, vx[i], vy[i], vx[j], vy[j]);
  }

  // draw stop
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2_uint_t lw = u8g2.getStrWidth("STOP");
  // Slightly lower baseline so it feels centered optically
  u8g2.drawStr(cx - lw/2, cy + 5, "STOP");
  u8g2.setDrawColor(1);
}

// sets up screen
void screenBegin(uint8_t i2cSDA, uint8_t i2cSCL) {
  GPIOManager& gpio = GPIOManager::getInstance();
  gpio.configureI2C(i2cSDA, i2cSCL, 400000); // 400kHz I2C
  Wire.begin(i2cSDA, i2cSCL);
  u8g2.setBusClock(400000); 
  u8g2.begin();
  u8g2.setPowerSave(0);
}

// 0 = STOP 1 = FORWARD 2 = LEFT 3 = RIGHT
void showDirection(int direction) {
  u8g2.clearBuffer();

  // Header banner
  u8g2.drawBox(0, 0, 128, 16);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_6x12_tf);
  drawCentered("DIRECTION", 12);
  u8g2.setDrawColor(1);

  const int cx = 64;
  const int cy = 34;      // icon center for arrows
  const int cyStop = 42;  
  const int labelY = 60;

  if (direction == 0) {
    drawStopOctagonFilled(cx, cyStop, 20);
  } 
  else if (direction == 1) {
    drawArrowUp(cx, cy);
    u8g2.setFont(u8g2_font_7x13B_tf);
    drawCentered("FORWARD", labelY);
  } 
  else if (direction == 2) {
    drawArrowLeft(cx, cy);
    u8g2.setFont(u8g2_font_7x13B_tf);
    drawCentered("LEFT", labelY);
  } 
  else if (direction == 3) {
    drawArrowRight(cx, cy);
    u8g2.setFont(u8g2_font_7x13B_tf);
    drawCentered("RIGHT", labelY);
  }

  u8g2.sendBuffer();
}
