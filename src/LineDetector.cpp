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
    return reading > config.sensors.whiteThreshold; // High values (>3000) = white
}

bool LineDetector::isSensorOnBlack(PhotoSensor& sensor) const {
    ControlConfig& config = ControlConfig::getInstance();
    int reading = sensor.readRaw();
    return reading < config.sensors.blackThreshold; // Low values (<10) = black
}

LineState LineDetector::detectLineState() {
    bool leftOnBlack = isSensorOnBlack(sensorL); // Left sensor
    bool centerOnBlack = isSensorOnBlack(sensorC); // Center sensor
    bool rightOnBlack = isSensorOnBlack(sensorR); // Right sensor
    
    // Priority 1: WBW = ON_LINE (center on black line - following normally)
    if (!leftOnBlack && centerOnBlack && !rightOnBlack) {
        return LineState::ON_LINE; // WBW = centered on line
    }
    
    // Priority 2: Turn Detection (two sensors on black)
    if (!leftOnBlack && centerOnBlack && rightOnBlack) {
        return LineState::TURN_RIGHT; // WBB = line moved right
    }
    if (leftOnBlack && centerOnBlack && !rightOnBlack) {
        return LineState::TURN_LEFT; // BBW = line moved left
    }
    
    // Priority 3: Single edge detection (edge of line detected)
    if (leftOnBlack && !centerOnBlack && !rightOnBlack) {
        return LineState::TURN_LEFT; // BWW = left edge detected, turn left
    }
    if (!leftOnBlack && !centerOnBlack && rightOnBlack) {
        return LineState::TURN_RIGHT; // WWB = right edge detected, turn right
    }
    
    // Priority 4: All black (intersection - treat as ON_LINE)
    if (leftOnBlack && centerOnBlack && rightOnBlack) {
        return LineState::ON_LINE; // BBB = intersection, go forward
    }
    
    // Priority 5: All other patterns = OFF_LINE (lost line)
    // This includes: WWW (no line), BWB (invalid)
    return LineState::OFF_LINE;
}

float LineDetector::calculateLinePosition() const {
    // Read all three sensors
    int leftRaw = sensorL.readRaw();   // Left sensor
    int centerRaw = sensorC.readRaw(); // Center sensor
    int rightRaw = sensorR.readRaw(); // Right sensor
    
    // Get thresholds from configuration; check for acutal values as they may differ from below
    ControlConfig& config = ControlConfig::getInstance();
    int blackThreshold = config.sensors.blackThreshold; // Low values (<10) = black
    int whiteThreshold = config.sensors.whiteThreshold; // High values (>3000) = white

    
    bool leftBlack = (leftRaw < blackThreshold);
    bool centerBlack = (centerRaw < blackThreshold);
    bool rightBlack = (rightRaw < blackThreshold);

    // Normalize each sensor reading to 0.0 (white) - 1.0 (black)
    // Map: low values (black) → 1.0, high values (white) → 0.0
    float lnorm = constrain(map(leftRaw, blackThreshold, whiteThreshold, 1000, 0), 0, 1000) / 1000.0f;
    float cnorm = constrain(map(centerRaw, blackThreshold, whiteThreshold, 1000, 0), 0, 1000) / 1000.0f;
    float rnorm = constrain(map(rightRaw, blackThreshold, whiteThreshold, 1000, 0), 0, 1000) / 1000.0f;

    // Discrete position mapping based on sensor patterns
    // -1.0 = line far left, 0.0 = centered, +1.0 = line far right
    
    if (leftBlack && centerBlack && rightBlack) {
        return 0.0f; // BBB = intersection, perfectly centered
    }
    
    if (!leftBlack && centerBlack && !rightBlack) {
        return 0.0f; // WBW = centered on line
    }
    
    if (leftBlack && centerBlack && !rightBlack) {
        return -0.5f; // BBW = line slightly left
    }
    
    if (!leftBlack && centerBlack && rightBlack) {
        return 0.5f; // WBB = line slightly right
    }
    
    if (leftBlack && !centerBlack && !rightBlack) {
        return -1.0f; // BWW = line far left (left edge only)
    }
    
    if (!leftBlack && !centerBlack && rightBlack) {
        return 1.0f; // WWB = line far right (right edge only)
    }
    
    if (!leftBlack && !centerBlack && !rightBlack) {
        return 0.0f; // WWW = no line detected, assume centered
    }
    
    // BWB pattern (shouldn't happen normally)
    if (leftBlack && !centerBlack && rightBlack) {
        return 0.0f; // BWB = ambiguous pattern, assume centered
    }
    
    return 0.0f; // Default to centered
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