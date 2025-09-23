#pragma once
#include "State.hpp"

class IdleState : public State {
    private:
        unsigned long waitStartTime;
        unsigned long waitDuration; // in milliseconds
        bool isWaiting;

    public:
        IdleState();

        void onEntry(ControlSubsystem* context) override;
        void onUpdate(ControlSubsystem* context) override;
        void onExit(ControlSubsystem* context) override;

        void setWaitDuration(int durationMs);
};