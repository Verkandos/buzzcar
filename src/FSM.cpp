#include <Arduino.h>
#include <FSM.hpp>
#include "ControlSubsystem.hpp"
#include "IdleState.hpp"
#include "ForwardState.hpp"
#include "TurnLeftState.hpp"
#include "TurnRightState.hpp"
#include "StopState.hpp"

FSM::FSM(ControlSubsystem* controlContext) : context(controlContext), currentState(nullptr) {
    // Initialize state instances
    idleState = std::make_unique<IdleState>();
    forwardState = std::make_unique<ForwardState>();
    turnLeftState = std::make_unique<TurnLeftState>();
    turnRightState = std::make_unique<TurnRightState>();
    stopState = std::make_unique<StopState>();
}

FSM::~FSM() {
    // Smart pointers handle cleanup automatically
}

void FSM::initialize() {
    // Start in IdleState
    currentState = std::move(idleState);
    currentState->onEntry(context);

    Serial.print("FSM initialized. Starting in IdleState ");
}

void FSM::update() {
    if (currentState) {
        currentState->onUpdate(context);
    }
}

void FSM::handleEvent(const Event& event) {
    if (!currentState) {
        Serial.println("FSM Error: No current state!");
        return;
    }

    // Log the event for debugging
    Serial.print("FSM: Processing event [");
    Serial.print(event.toString());
    Serial.print("] in state [");
    Serial.print(getCurrentStateName());
    Serial.println("]");

    // Validate transition
    if (!isValidTransition(event.getType())) {
        Serial.print("FSM Warning: Invalid transition for event [");
        Serial.print(event.toString());
        Serial.print("] in state [");
        Serial.print(getCurrentStateName());
        Serial.println("]");
        return;
    }

    // Handle state transitions based on current state and event
    switch(event.getType()) {
        case EventType::START:
            if (isInState("StopState")) {
                transitionTo(std::move(idleState));
            }
            break;
        
        
        case EventType::STOP:
             // Any state can transition to STOP
            transitionTo(std::move(stopState));
            break;
        
        case EventType::START_MOVEMENT:
            if (isInState("IdleState")) {
                transitionTo(std::move(forwardState));
            }
            break;
        
        case EventType::TURN_LEFT:
            if (isInState("ForwardState")) {
                transitionTo(std::move(turnLeftState));
            }
            break;
        
        case EventType::TURN_RIGHT:
            if (isInState("ForwardState")) {
                transitionTo(std::move(turnRightState));
            }
            break;
        
        case EventType::FORWARD:
            if (isInState("TurnLeftState") || isInState("TurnRightState")) {
                transitionTo(std::move(forwardState));
            }
            break;
        
        case EventType::OFF_LINE:
            // Off-line detection triggers stop from any active state
            if (!isInState("StopState") && !isInState("IdleState")) {
                transitionTo(std::move(stopState));
            }
            break;

        default:
            Serial.print("FSM: Unhandled event type: ");
            Serial.print(event.toString());
            break;
    }
}

void FSM::transitionTo(std::unique_ptr<State> newState) {
    if (!newState) {
        Serial.println("FSM Error: Attempted to transition to null state!");
        return;
    }
    // Exit current state
    if (currentState) {
        Serial.print("FSM: Exiting ");
        Serial.println(getCurrentStateName());
        currentState->onExit(context);
    }

    // Transition to new state
    currentState = std::move(newState);

    Serial.print("FSM: Entering ");
    Serial.println(getCurrentStateName());
    currentState->onEntry(context);

    // Re-instantiate state instances for future transitions
    if (currentState.get() == idleState.get()) {
        idleState = std::make_unique<IdleState>();
    } else if (currentState.get() == forwardState.get()) {
        forwardState = std::make_unique<ForwardState>();
    } else if (currentState.get() == turnLeftState.get()) {
        turnLeftState = std::make_unique<TurnLeftState>();
    } else if (currentState.get() == turnRightState.get()) {
        turnRightState = std::make_unique<TurnRightState>();
    } else if (currentState.get() == stopState.get()) {
        stopState = std::make_unique<StopState>();
    }
}

bool FSM::isValidTransition(EventType eventType) const {
    // Define valid transitions based on current state
    const char* currentStateName = getCurrentStateName();

    switch (eventType) {
        case EventType::STOP:
            return true; // STOP is always valid (emergency)
        
        case EventType::START:
            return isInState("StopState");

        case EventType::START_MOVEMENT:
            return isInState("IdleState");
        
        case EventType::TURN_LEFT:
        case EventType::TURN_RIGHT:
            return isInState("ForwardState");

        case EventType::FORWARD:
            return isInState("TurnLeftState") || isInState("TurnRightState");
        
        case EventType::OFF_LINE:
            return !isInState("StopState") && !isInState("IdleState");

        default:
            return false; // Unknown events are invalid
    }
}

State* FSM::getCurrentState() const {
    return currentState.get();
}

const char* FSM::getCurrentStateName() const {
    if (!currentState) {
        return "NULL";
    }

    // Identify current state stype
    if (dynamic_cast<IdleState*>(currentState.get())) {
        return "IdleState";
    } else if (dynamic_cast<ForwardState*>(currentState.get())) {
        return "ForwardState";
    } else if (dynamic_cast<TurnLeftState*>(currentState.get())) {
        return "TurnLeftState";
    } else if (dynamic_cast<TurnRightState*>(currentState.get())) {
        return "TurnRightState";
    } else if (dynamic_cast<StopState*>(currentState.get())) {
        return "StopState";
    } else {
        return "UnknownState";
    }
}

void FSM::reset() {
    if (currentState) {
        currentState->onExit(context);
    }

    // Reset to IdleState
    currentState = std::move(idleState);
    currentState->onEntry(context);
    idleState = std::make_unique<IdleState>();

    Serial.println("FSM has been reset to IdleState.");
}

bool FSM::isInState(const char* stateName) const {
    return strcmp(getCurrentStateName(), stateName) == 0;
}