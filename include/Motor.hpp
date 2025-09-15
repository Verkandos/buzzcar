#pragma once
#include "Actuator.hpp"

class Motor : public Actuator {
    private:
        int pinPWM;
        int currentPWM;
    public:
        Motor();
        void setPWM(int pwm);
        int getPWM() const;
        void activate() override;
        void stop();
};