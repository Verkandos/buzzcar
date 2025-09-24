#pragma once
#include "State.hpp"
#include "Event.hpp"
#include <vector>
#include <memory>

class ControlSubsystem;

class FSM {
    private:
        std::unique_ptr<State> currentState;
        ControlSubsystem* context;

        // State instances (managed by FSM)
        std::unique_ptr<State> idleState;
        std::unique_ptr<State> forwardState;
        std::unique_ptr<State> turnLeftState;
        std::unique_ptr<State> turnRightState;
        std::unique_ptr<State> stopState;

        // Internal methods
        void transitionTo(std::unique_ptr<State> newState);
        bool isValidTransition(EventType eventType) const;

    public:
        FSM(ControlSubsystem* controlContext);
        ~FSM();

        // Core FSM methods
        void initialize();
        void update();
        void handleEvent(const Event& event);

        // State management
        State* getCurrentState() const;
        const char* getCurrentStateName() const;

        // Utility methods
        void reset();
        bool isInState(const char* stateName) const;
};
