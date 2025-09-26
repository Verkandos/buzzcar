#pragma once
#include "State.hpp"
#include "PIDController.hpp"

class ControlSubsytem;

class ForwardState : public State {
    private:
        int baseSpeed; // Base speed for both motors (0-100%)
        PIDController pidController; // PID controller for line following
    public:
        ForwardState();
        ~ForwardState() override = default;

        void onEntry(ControlSubsystem* controlSys) override;
        void onUpdate(ControlSubsystem* controlSys) override;
        void onExit(ControlSubsystem* controlSys) override;

        // PID specific methods
        
        void setPIDGains(float p, float i, float d);
        void setBaseSpeed(int speed);
};