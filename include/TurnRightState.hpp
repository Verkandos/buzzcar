#pragma once
#include "State.hpp"

class TurnRightState : public State {
public:
    TurnRightState() : State("TurnRight") {}
    
    void onEntry() override;
    void onUpdate() override;
    void onExit() override;
};