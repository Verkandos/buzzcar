#pragma once
#include "State.hpp"

class IdleState : public State {
    public:
        IdleState() : State("Idle") {}
    
        void onEntry() override;
        void onUpdate() override;
        void onExit() override;
};