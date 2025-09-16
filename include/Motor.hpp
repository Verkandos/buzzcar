#pragma once
#include "Actuator.hpp"

class Motor : public Actuator {
    private:
        int pinPWM;
        int currentSpeedPercent;
        int minPWM;
        int maxPWM;
        int minimumStartPWM;
        
        void applyPWM(int pwmValue);
    public:
        Motor();
        Motor(int pin);

        // Initiliazation
        void initialize();
        void initialize(int pin);
        
        // Actuator interface
        void activate() override;

        // Motor Control Logic
        void setSpeed(int speedPercent);
        void setPWMRange(int min, int max);
        void setMinimumStartPWM(int pwm);
        void stop();

        // Status/Feedback for PID
        int getCurrentSpeed() const;
        int getCurrentPWM() const;
        int getMinimumStartPWM() const;
        int getMaxPWM() const;
        int getMinPWM() const;
        bool isRunning() const;
};