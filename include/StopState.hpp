#pragma once
#include "State.hpp"

class StopState : public State {
public:
    StopState() : State("Stop") {}
    
    void onEntry() override;
    void onUpdate() override;
    void onExit() override;
};