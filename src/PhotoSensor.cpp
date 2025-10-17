#include "Arduino.h"
#include "PhotoSensor.hpp"
#include "GPIOManager.hpp"
#include "ControlConfig.hpp"

PhotoSensor::PhotoSensor() 
    : pinAnalog(-1), lineThreshold(512), currentRawValue(0),
     calibrationMin(4095), calibrationMax(0), isCalibrated(false) {
        // Default constructor
}

PhotoSensor::PhotoSensor(int pin, int threshold) 
    : pinAnalog(pin), lineThreshold(threshold), currentRawValue(0),
      calibrationMin(4095), calibrationMax(0), isCalibrated(false) {
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
    ControlConfig& config = ControlConfig::getInstance();
    // Get raw ADC value from GPIOManager (no processing)
    if (pinAnalog == -1) return 0;

    // Use singleton instance
    GPIOManager& gpio = GPIOManager::getInstance();
    // currentRawValue = gpio.readAnalog(pinAnalog);
    
    // Multisampling: Take 8 samples and average to reduce noise
    // ESP32-C6 ADC is sensitive to noise, multisampling helps mitigate this
    long sum = 0;
    
    for (int i = 0; i < config.sensors.samplingCount; i++) {
        sum += gpio.readAnalog(pinAnalog);
        delayMicroseconds(10); // Small delay between samples 
    }
    
    currentRawValue = sum / config.sensors.samplingCount;
    return currentRawValue;
}

float PhotoSensor::readNormalized() {
    // Sensor logic: Normalize to 0.0 - 1.0 based on range
    int rawValue = readRaw();

    if(isCalibrated) {
        // Use calibrated range
        return constrain(map(rawValue, calibrationMin, calibrationMax, 0, 1000), 0, 1000) / 1000.0f;
    } else {
        // Use full ADC range (0-4095 for ESP32)
        return rawValue / 4095.0f;
    }
}

bool PhotoSensor::detectLine() {
    // Sensor logiC: line detection based on threshold
    int rawValue = readRaw();
    return rawValue > lineThreshold;
}

bool PhotoSensor::detectDarkSurface() {
    return detectLine();
}

bool PhotoSensor::detectLightSurface() {
    return !detectLine();
}

void PhotoSensor::setLineThreshold(int threshold) {
    lineThreshold = constrain(threshold, 0, 4095);
}

void PhotoSensor::calibrate(int durationMs) {
    // Sensor logic: Auto-calibration routine
    Serial.println("Calibrating sensor... Move over different surfaces");

    calibrationMin = 4095;
    calibrationMax = 0;

    unsigned long startTime = millis();
    while(millis() - startTime < (unsigned long)durationMs) {
        int reading  = readRaw();

        if(reading < calibrationMin) calibrationMin = reading;
        if(reading > calibrationMax) calibrationMax = reading;

        delay(10); // Sample every 10ms
    }
    isCalibrated = true;

    lineThreshold = (calibrationMin + calibrationMax) / 2;
    Serial.print("Calibration complete. Min: "); Serial.print(calibrationMin);
    Serial.print(" Max: "); Serial.print(calibrationMax);
    Serial.print(" Threshold set to: "); Serial.println(lineThreshold);
}

void PhotoSensor::setCalibration(int minValue, int maxValue) {

    // Sensor logic: Manual calibration setting
    calibrationMin = constrain(minValue, 0, 4095);
    calibrationMax = constrain(maxValue, 0, 4095);
    isCalibrated = true;

    // set threshold to midpoint
    lineThreshold = (calibrationMin + calibrationMax) / 2;
}

int PhotoSensor::getThreshold() const {
    return lineThreshold;
}

int PhotoSensor::getCurrentReading() const {
    return currentRawValue;
}

bool PhotoSensor::isCalibrationComplete() const {
    return isCalibrated;
}