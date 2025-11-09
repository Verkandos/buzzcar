#pragma once
#include "Event.hpp"
#include "State.hpp"
#include <memory>

// Forward Declaration
class ControlSubsystem;

class FSM {
    private:
        std::unique_ptr<State> currentState;
        ControlSubsystem* context;

    public:
        FSM(ControlSubsystem* controlContext);
        ~FSM();
        
        // Initialize FSM - starts in IdleState
        void initialize();
        
        // Handle events and perform state transitions
        void handleEvent(const Event& event);
        
        // Transition to a new state
        void transitionTo(State* newState);
        
        // Update current state
        void update();
        
        // Get current state
        State* getCurrentState() const;
        
        // Get current state name for debugging
        const char* getCurrentStateName() const;
        
        // Reset FSM to initial state
        void reset();
        // Check if FSM is in a specific state
        bool isInState(const char* stateName) const;

};