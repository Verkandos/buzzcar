#include "Arduino.h"
#include "PhotoSensor.hpp"
#include "GPIOManager.hpp"

PhotoSensor::PhotoSensor() 
    : pinAnalog(-1), lineThreshold(512), currentRawValue(0) {
        // Default constructor;
}

PhotoSensor::PhotoSensor(int pin, int threshold) 
    : pinAnalog(pin), lineThreshold(threshold), currentRawValue(0) {
        // Constructor with pin and threshold
}

void PhotoSensor::initialize() {
    if (pinAnalog != -1) {
        // Use singleton instance
        GPIOManager& gpio = GPIOManager::getInstance();
        gpio.configurePin(pinAnalog, "analog_input");
    }
}

void PhotoSensor::initialize(int pin) {
    pinAnalog = pin;
    initialize();
}

int PhotoSensor::readRaw() {
    // Get raw ADC value from GPIOManager (no processing)
    if (pinAnalog == -1) return 0;

    // Use singleton instance
    GPIOManager& gpio = GPIOManager::getInstance();
    currentRawValue = gpio.readAnalog(pinAnalog);
    return currentRawValue;
}


void PhotoSensor::setLineThreshold(int threshold) {
    lineThreshold = constrain(threshold, 0, 4095);
}


int PhotoSensor::getThreshold() const {
    return lineThreshold;
}