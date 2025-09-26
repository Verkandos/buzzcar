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
}

FSM::~FSM() {
    // Smart pointers handle cleanup automatically
}

void FSM::initialize() {
    // Start in IdleState
    currentState = std::make_unique<IdleState>();
    currentState->onEntry(context);
    
    Serial.println("FSM initialized - Starting in IdleState");
}

void FSM::update() {
    if (currentState) {
        currentState->onUpdate(context);
    }
}

void FSM::handleEvent(const Event& event) {
    switch(event.getType()) {
        case EventType::START:
            if (getCurrentStateName() == "StopState") {
                transitionTo(new IdleState());
            }
            break;
        
        case EventType::STOP:
            transitionTo(new StopState());
            break;
        
        case EventType::START_MOVEMENT:
            if (getCurrentStateName() == "ForwardState") {
                transitionTo(new ForwardState());
            }
            break;

        case EventType::TURN_RIGHT:
            if (getCurrentStateName() == "ForwardState") {
                transitionTo(new TurnRightState());
            }
            break;
            
        case EventType::FORWARD:
            if (getCurrentStateName() == "TurnLeftState" || 
                getCurrentStateName() == "TurnRightState") {
                transitionTo(new ForwardState());
            }
            break;
        
        case EventType::OFF_LINE:
            if (getCurrentStateName() != "StopState" && 
                getCurrentStateName() != "IdleState") {
                transitionTo(new StopState());
            }
            break;
            
        default:
            break;
    }
}

void FSM::transitionTo(State* newState) {
    // Exit current state
    if (currentState) {
        Serial.print("FSM: Exiting ");
        Serial.println(getCurrentStateName());
        currentState->onExit(context);
        currentState.reset(); // Clear current state
    }

    // Transition to new state
    currentState.reset(newState);
    currentState->onEntry(context);
}

bool FSM::isValidTransition(EventType eventType) const {
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
            return false;
    }
}

State* FSM::getCurrentState() const {
    return currentState.get();
}

const char* FSM::getCurrentStateName() const {
    if (!currentState) {
        return "NULL";
    }
    return currentState->getName().c_str();
}

void FSM::reset() {
    if (currentState) {
        currentState->onExit(context);
    }

    // Reset to IdleState
    currentState = std::make_unique<IdleState>();
    currentState->onEntry(context);

    Serial.println("FSM has been reset to IdleState.");
}

bool FSM::isInState(const char* stateName) const {
    return strcmp(getCurrentStateName(), stateName) == 0;
}