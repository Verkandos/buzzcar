#include <Arduino.h>
#include "TurnRightState.hpp"
#include "ControlSubsystem.hpp"
#include "Event.hpp"
#include "FSM.hpp"
#include "ControlConfig.hpp"

TurnRightState::TurnRightState() : State("TurnRightState"), baseSpeed(50), turnSpeed(30), pidController(1.5f, 0.05f, 0.8f) {
    // Initialize with conservative turn speeds and PID gains
    pidController.setOutputLimits(-15.0f, 15.0f); // smaller corrections during turns
}

void TurnRightState::onEntry(ControlSubsystem* context) {
    ControlConfig& config = ControlConfig::getInstance();
    // Initialize with conservative turn speeds and PID gains
    pidController.reset();

    // Start turning right: left motor forward, right motor stopped
    context->getMotorA()->setSpeed(config.motor.turnSpeed); // Left motor at turn speed
    context->getMotorB()->setSpeed(0); // Right motor stopped

}

void TurnRightState::onUpdate(ControlSubsystem* context) {
    ControlConfig& config = ControlConfig::getInstance();
    
    // Maintain turn - right motor off, left motor turning
    // Use PID for fine-tuning if still detecting turn state
    LineState currentState = context->getLineDetector()->detectLineState();
    
    if (currentState == LineState::TURN_RIGHT) {
        // Fine-tune turn speed with PID
        float linePosition = context->getLineDetector()->calculateLinePosition();
        float pidOutput = pidController.compute(0.5f, linePosition); // Target is +0.5 for right turn
        
        int leftMotorSpeed = config.motor.turnSpeed + (int)pidOutput;
        leftMotorSpeed = constrain(leftMotorSpeed, 10, 80);
        
        context->getMotorA()->setSpeed(leftMotorSpeed);
        context->getMotorB()->setSpeed(0);
    } else {
        // Maintain basic turn configuration - let generateEvent() handle state transitions
        context->getMotorA()->setSpeed(config.motor.turnSpeed);
        context->getMotorB()->setSpeed(0);
    }
}

void TurnRightState::onExit(ControlSubsystem* context) {
    // Stop motors when exiting turn state
    context->getMotorA()->setSpeed(0);
    context->getMotorB()->setSpeed(0);
}