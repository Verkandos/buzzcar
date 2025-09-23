#pragma once
#include "State.hpp"
#include "PIDController.hpp"

class ControlSubsystem;

class TurnRightState : public State {
    private:
        int baseSpeed; // Base speed for motors
        int turnSpeed; // Speed during the turn
        PIDController pidController; // PID controller for turn fine-tuning
public:
    TurnRightState();

    void onEntry(ControlSubsystem* context) override;
    void onUpdate(ControlSubsystem* context) override;
    void onExit(ControlSubsystem* context) override;

    // Configuration methods
    void setTurnSpeed(int speed);
    void setPIDGains(float p, float i, float d);
};