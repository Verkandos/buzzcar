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
    ~TurnRightState() override = default;

    void onEntry(ControlSubsystem* controlSys) override;
    void onUpdate(ControlSubsystem* controlSys) override;
    void onExit(ControlSubsystem* controlSys) override;
};