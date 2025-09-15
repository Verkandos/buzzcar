#pragma once
#include "State.hpp"
#include "Event.hpp"


class ControlSubsystem {
    private:
        State* currentState;
        float error;
        int Tmin;
        int Tmax;
    
    public:
        ControlSubsystem();

        void initialize();
        float computeError();
        Event generateEvent();
        Event handleEvent();
        State* getCurrentState();

};