#pragma once
#include "State.hpp"

class ControlSubsystem;

class IdleState : public State {
    private:
        unsigned long waitStartTime;
        unsigned long waitDuration; // in milliseconds
        bool isWaiting;

    public:
        IdleState();
        ~IdleState() override = default;

        void onEntry(ControlSubsystem* controlSys) override;
        void onUpdate(ControlSubsystem* controlSys) override;
        void onExit(ControlSubsystem* controlSys) override;
};