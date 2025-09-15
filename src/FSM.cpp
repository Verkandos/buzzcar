#include <Arduino.h>
#include <FSM.hpp>

void FSM::initialize(std::vector<State*>& states, State* initialState) {
    // Store the staes vector
    this->states = states;

    // Set the initial state
    currentState = initialState;

    // TODO: Call onEntry of the initial state
    if (currentState) {
        // currentState->onEntry();
    }
}

void FSM::transition(const Event& event) {
    // TODO: Handle state transitions based on the events
    // Logic to determine the next state based on the current state and event

    if (currentState != nullptr) {
        // currentState->onExit();
        // Determine next state based on event
    }
}

State* FSM::getCurrentState() const {
    return currentState;
}

void FSM::update() {
    if (currentState) {
        currentState->onUpdate();
    }
}

void FSM::reset() {
    // Reset the FSM to its initial state
    if (!states.empty() && currentState != nullptr) {
        // TODO: Reset to first state or other designated initial state
    }
}