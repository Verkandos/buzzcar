#include "Arduino.h"
#include "PhotoSensor.hpp"

PhotoSensor::PhotoSensor() 
    : pinAnalog(-1), threshold(512), currentAnalogValue(0), currentDigitalValue(false) {
        // Default constructor
}

PhotoSensor::PhotoSensor(int pin, int digitalThreshold) 
    : pinAnalog(pin), threshold(digitalThreshold), currentAnalogValue(0), currentDigitalValue(false) {
        // Constructor with pin and threshold
}

void PhotoSensor::initialize(int pin) {
    // TODO: Initialize the photosensor analog pin
    pinAnalog = pin;
}

float PhotoSensor::readValue() {
    return value;
}

bool PhotoSensor::readDigital() {
    currentDigitalValue = (analogRead(pinAnalog) > threshold);
    return currentDigitalValue;
}

int PhotoSensor::readAnalog() {
    // TODO: return raw analog value from the sensor
    return currentAnalogValue;
}

void PhotoSensor::setThreshold(int threshold) {
    this->threshold = threshold;
}