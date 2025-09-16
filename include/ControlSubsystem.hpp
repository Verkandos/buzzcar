#pragma once
#include "State.hpp"
#include "Event.hpp"
#include "FSM.hpp"
#include <vector>


class ControlSubsystem {
    private:
        State* currentState;
        float error;
        int Tmin;
        int Tmax;
        FSM fsm;
    
    public:
        ~ControlSubsystem();
        ControlSubsystem();

        void initialize();
        void update();
        float computeError();
        Event generateEvent();
        Event handleEvent();
        State* getCurrentState();

};