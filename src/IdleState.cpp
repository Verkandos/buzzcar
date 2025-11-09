#include <Arduino.h>
#include "IdleState.hpp"
#include "ControlSubsystem.hpp"
#include "Event.hpp"
#include "FSM.hpp"

IdleState::IdleState() : State("IdleState"), waitStartTime(0), waitDuration(3000), isWaiting(false) {
    // Initialize with base State name and member variables
}

void IdleState::onEntry(ControlSubsystem* context) {
    // Ensure motors are stopped
    context->getMotorA()->setSpeed(0);
    context->getMotorB()->setSpeed(0);

    // Start waiting period
    waitStartTime = millis();
    isWaiting = true;

    // Optionally, calibration code can be added here
}

void IdleState::onUpdate(ControlSubsystem* context) {
    // TODO: Code to execute during the Idle state
    if (isWaiting) {
        // Check if wait period is complete
        if (millis() - waitStartTime >= waitDuration) {
            isWaiting = false;
            // Transition to Forward state after waiting
            context->getFSM()->handleEvent(Event(EventType::START_MOVEMENT));
        }
    }
    // If not waiting, just remain idle until event triggers transition
}

void IdleState::onExit(ControlSubsystem* context) {
    isWaiting = false; // Reset waiting flag
}