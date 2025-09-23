#include <Arduino.h>
#include "IdleState.hpp"
#include "ControlSubsystem.hpp"
#include "Event.hpp"

IdleState::IdleState() : waitStartTime(0), waitDuration(3000), isWaiting(false) {
    // Wait for 2 seconds before transitioning to Forward state
}

void IdleState::onEntry(ControlSubsystem* context) {
    // Ensure motors are stopped
    context->getMotorA().setSpeed(0);
    context->getMotorB().setSpeed(0);

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
            context->getFSM().handleEvent(Event(EventType::START));
        }
    }
    // If not waiting, just remain idle until event triggers transition
}

void IdleState::onExit(ControlSubsystem* context) {
    isWaiting = false; // Reset waiting flag
}

void IdleState::setWaitDuration(int durationMs) {
    waitDuration = durationMs;
}