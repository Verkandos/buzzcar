#pragma once
#include <Arduino.h>

void speakerBegin(uint8_t audioPin, uint8_t ledcChannel, uint8_t resolutionBits = 10, uint8_t dutyPercent = 50);

void startMelodyForDirection(int direction);

void serviceMelody();

void speakerStop();
