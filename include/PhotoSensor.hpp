#pragma once

class PhotoSensor {
    private:
        int pinAnalog;
        int lineThreshold;
        int currentRawValue;
        int calibrationMin;
        int calibrationMax;
        bool isCalibrated;
        
    public:
        PhotoSensor();
        PhotoSensor(int pin, int threshold = 2000);
        
        // Initialization
        void initialize();
        void initialize(int pin);
        
        // Reading Methods
        int readRaw();                    // Raw ADC value
        float readNormalized();           // 0.0-1.0 normalized
        bool detectLine();                // Line detection
        bool detectDarkSurface();         // Dark surface
        bool detectLightSurface();        // Light surface
        
        // Configuration & Calibration
        void setLineThreshold(int threshold);
        void calibrate(int durationMs = 3000);
        void setCalibration(int minValue, int maxValue);
        
        // Status
        int getThreshold() const;
        int getCurrentReading() const;
        bool isCalibrationComplete() const;
};