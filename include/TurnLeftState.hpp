#pragma once
#include "State.hpp"

class TurnLeftState : public State {
public:
    TurnLeftState() : State("TurnLeft") {}
    
    void onEntry() override;
    void onUpdate() override;
    void onExit() override;
};