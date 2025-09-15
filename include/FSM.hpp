#pragma once
#include "State.hpp"
#include "Event.hpp"
#include <vector>


class FSM {
    private:
        std::vector<State*> states;
        State* currentState = nullptr;
    public:
        void initialize(std::vector<State*>& states, State* initialState);
        void transition(const Event& event);
        State* getCurrentState() const;
        void update();
        void reset();
};
