#pragma once
#include "State.hpp"

class ForwardState : public State {
    private:
        // PID Controller variables
        float kp, ki, kd;
        float previousError;
        float integral;
        float setpoint;
    public:
        ForwardState() : State("Forward"), kp(1.0f), ki(0.0), kd(0.0f), 
        previousError(0.0f), integral(0.0f), setpoint(0.0f) {}
    
        void onEntry() override;
        void onUpdate() override;
        void onExit() override;

        // PID specific methods
        void setPIDGains(float p, float i, float d);
        void setSetpoint(float target);
        float computePID(float currentValue);
};