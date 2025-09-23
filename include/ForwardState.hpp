#pragma once
#include "State.hpp"
#include "PIDController.hpp"

class ForwardState : public State {
    private:
        int baseSpeed; // Base speed for both motors (0-100%)
        PIDController pidController; // PID controller for line following
    public:
        ForwardState();

        void onEntry(ControlSubsystem* context);
        void onUpdate(ControlSubsystem* context);
        void onExit(ControlSubsystem* context);

        // PID specific methods
        
        void setPIDGains(float p, float i, float d);
        void setBaseSpeed(int speed);
};