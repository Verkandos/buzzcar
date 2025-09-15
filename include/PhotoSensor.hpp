#pragma once
#include "Sensor.hpp"

class PhotoSensor : public Sensor {
    private:
        int pinAnalog;
        int threshold; // Threshold for analog-to-digital conversion
        int currentAnalogValue;
        bool currentDigitalValue;

    public:
        PhotoSensor();
        PhotoSensor(int pin, int digitalThreshold = 512);
        void initialize(int pin) override;
        float readValue() override;

        bool readDigital();
        int readAnalog();
        void setThreshold(int threshold);
        int getThreshold() const {return threshold;};
};