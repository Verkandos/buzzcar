#pragma once
#include "PhotoSensor.hpp"

enum class LineState {
    ON_LINE, // Mid sensor = black, Side sensors = white
    TURN_LEFT, // Left sensor = black
    TURN_RIGHT, // Right sensor = black
    OFF_LINE, // All sensors = white
    UNKNOWN // All sensors = black or inconsistent readings
};

class LineDetector {
    private:
        PhotoSensor& sensorL; // Left sensor
        PhotoSensor& sensorC; // Center sensor
        PhotoSensor& sensorR; // Right sensor
        
        int blackThreshold; // Threshold for black detection
        int whiteThreshold; // Threshold for white detection

        bool isSensorOnBlack(PhotoSensor& sensor) const;
        bool isSensorOnWhite(PhotoSensor& sensor) const;
        
    public:

        LineDetector(PhotoSensor& left, PhotoSensor& center, PhotoSensor& right,
                     int blackThresh = 2000, int whiteThresh = 3000);

        LineState detectLineState();
        float calculateLinePosition() const; // -1.0 (left) to 1.0 (right)
        void setThresholds(int blackThreshold, int whiteThreshold);
        void getThresholds(int& blackThreshold, int& whiteThreshold) const;
        void getRawReadings(int& left, int& center, int& right) const;
        
};

