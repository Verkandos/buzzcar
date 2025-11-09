#pragma once

class PhotoSensor {
    private:
        int pinAnalog;
        int lineThreshold;
        int currentRawValue;
        
    public:
        PhotoSensor();
        PhotoSensor(int pin, int threshold = 2000);
        
        // Initialization
        void initialize();
        void initialize(int pin);
        
        // Reading Methods
        int readRaw();                    // Raw ADC value
        
        // Configuration
        void setLineThreshold(int threshold);
        
        // Status
        int getThreshold() const;
};