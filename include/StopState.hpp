#pragma once
#include "State.hpp"

class ControlSubsystem;

class StopState : public State {
public:
    void onEntry(ControlSubsystem* context) override;
    void onUpdate(ControlSubsystem* context) override;
    void onExit(ControlSubsystem* context) override;
};