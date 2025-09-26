#pragma once
#include "State.hpp"

class ControlSubsystem;

class StopState : public State {
    public:
        StopState() : State("StopState") {}
        ~StopState() override = default;
        
        void onEntry(ControlSubsystem* controlSys) override;
        void onUpdate(ControlSubsystem* controlSys) override;
        void onExit(ControlSubsystem* controlSys) override;
};