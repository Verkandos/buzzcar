#include "LineDetector.hpp"
#include "ControlConfig.hpp"
#include <Arduino.h>

LineDetector::LineDetector(PhotoSensor& left, PhotoSensor& center, PhotoSensor& right,
                           int blackThreshold, int whiteThreshold)
    : sensorL(left), sensorC(center), sensorR(right),
      blackThreshold(blackThreshold), whiteThreshold(whiteThreshold) {
    // Constructor
}

bool LineDetector::isSensorOnWhite(PhotoSensor& sensor) const {
    ControlConfig& config = ControlConfig::getInstance();
    int reading = sensor.readRaw();
    return reading < config.sensors.whiteThreshold; // Low values (0-100) = white
}

bool LineDetector::isSensorOnBlack(PhotoSensor& sensor) const {
    ControlConfig& config = ControlConfig::getInstance();
    int reading = sensor.readRaw();
    return reading > config.sensors.blackThreshold; // High values (3000+) = black
}

LineState LineDetector::detectLineState() {
    bool leftOnBlack = isSensorOnBlack(sensorL); // Left sensor
    bool centerOnBlack = isSensorOnBlack(sensorC); // Center sensor
    bool rightOnBlack = isSensorOnBlack(sensorR); // Right sensor
    
    // Priority 1: Turn Detection
    if (leftOnBlack && centerOnBlack && !rightOnBlack) {
        return LineState::TURN_LEFT; // Left and center on black, right on white
    }
    if (!leftOnBlack && centerOnBlack && rightOnBlack) {
        return LineState::TURN_RIGHT; // Right and center on black, left on white
    }

    // Priority 2: On Line
    if (!leftOnBlack && centerOnBlack && !rightOnBlack) {
        return LineState::ON_LINE; // Only center on black
    }
    
    // Priority 3: Off Line
    if (!centerOnBlack) {
        return LineState::OFF_LINE; // All on white
    }

    // Priority 4: Unknown
    // If none of the above conditions are met, the line state is unknown
    if (leftOnBlack && centerOnBlack && rightOnBlack) {
        return LineState::UNKNOWN; // All sensors on black
    }

    // Default to UNKNOWN for any other inconsistent readings
    return LineState::UNKNOWN;
}

float LineDetector::calculateLinePosition() const {
    // Read all three sensors
    int leftRaw = sensorL.readRaw();   // Left sensor
    int centerRaw = sensorC.readRaw(); // Center sensor
    int rightRaw = sensorR.readRaw(); // Right sensor
    
    // Get thresholds from configuration; check for acutal values as they may differ from below
    ControlConfig& config = ControlConfig::getInstance();
    int blackThreshold = config.sensors.blackThreshold; // Default 3000;
    int whiteThreshold = config.sensors.whiteThreshold; // Default 100 

    // Normallize each sensor reading to 0.0 (white) - 1.0 (black)
    float lnorm = constrain(map(leftRaw, whiteThreshold, blackThreshold, 0, 1000), 0, 1000) / 1000.0f;
    float cnorm = constrain(map(centerRaw, whiteThreshold, blackThreshold, 0, 1000), 0, 1000) / 1000.0f;
    float rnorm = constrain(map(leftRaw, whiteThreshold, blackThreshold, 0, 1000), 0, 1000) / 1000.0f;

    // Calculate total "line darkness" 
    float totalWeight = lnorm + cnorm + rnorm;
    
    float position = (-1.0f * lnorm + 0.0f * cnorm + 1.0f * rnorm) / (totalWeight);

    position = constrain(position, -1.0f, 1.0f);

    return position;
}

void LineDetector::setThresholds(int blackThreshold, int whiteThreshold) {
    // Validate thresholds
    if (blackThreshold < whiteThreshold && blackThreshold >= 0 && whiteThreshold <= 4095) {
        this->blackThreshold = blackThreshold;
        this->whiteThreshold = whiteThreshold;
    }
    // If validation fails, do not update thresholds
}

void LineDetector::getThresholds(int& blackThreshold, int& whiteThreshold) const {
    blackThreshold = this->blackThreshold;
    whiteThreshold = this->whiteThreshold;
}

void LineDetector::getRawReadings(int& left, int& center, int& right) const {
    left = sensorL.readRaw();
    center = sensorC.readRaw();
    right = sensorR.readRaw();
}