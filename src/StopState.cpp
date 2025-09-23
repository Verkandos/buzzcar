#include <Arduino.h>
#include "StopState.hpp"
#include "ControlSubsystem.hpp"

void StopState::onEntry(ControlSubsystem* context) {
    // Immediately stop all motors when entering the Stop state
    context->getMotorA().setSpeed(0);
    context->getMotorB().setSpeed(0);

    // context->getUserInterface().setSystemOff(); // Update UI to reflect stopped state;
}

void StopState::onUpdate(ControlSubsystem* context) {
    // Stay stopped, do nothing
    // Transition back to IdleState happens via ControlSubsystem
    // when button is pressed again
}

void StopState::onExit(ControlSubsystem* context) {
    // No specific cleanup needed
}