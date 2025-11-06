#include <Arduino.h>
#include "TurnLeftState.hpp"
#include "ControlSubsystem.hpp"
#include "Event.hpp"
#include "FSM.hpp"
#include "ControlConfig.hpp"
#include "Screen.h"

TurnLeftState::TurnLeftState() : State("TurnLeftState"), baseSpeed(50), turnSpeed(30), pidController(1.5f, 0.05f, 0.8f) {
    // Initialize with conservative turn speeds and PID gains
    pidController.setOutputLimits(-15.0f, 15.0f); // smaller corrections during turns
}

void TurnLeftState::onEntry(ControlSubsystem* context) {
    ControlConfig& config = ControlConfig::getInstance();
    // Reset PID controller for the turn
    pidController.reset();

    // Start turning left by reducing left motor speed
    context->getMotorA()->setSpeed(0); // Left motor slower
    context->getMotorB()->setSpeed(config.motor.turnSpeed); // Right motor at turn speed
}
void TurnLeftState::onUpdate(ControlSubsystem* context) {
    ControlConfig& config = ControlConfig::getInstance();
    
    // Maintain turn - left motor off, right motor turning
    // Use PID for fine-tuning if still detecting turn state
    LineState currentState = context->getLineDetector()->detectLineState();
    
    if (currentState == LineState::TURN_LEFT) {
        // Fine-tune turn speed with PID
        float linePosition = context->getLineDetector()->calculateLinePosition();
        float pidOutput = pidController.compute(-0.5f, linePosition); // Target is -0.5 for left turn
        
        int rightMotorSpeed = config.motor.turnSpeed + (int)pidOutput;
        rightMotorSpeed = constrain(rightMotorSpeed, 10, 80);
        
        context->getMotorA()->setSpeed(0);
        context->getMotorB()->setSpeed(rightMotorSpeed);
    } else {
        // Maintain basic turn configuration - let generateEvent() handle state transitions
        context->getMotorA()->setSpeed(0);
        context->getMotorB()->setSpeed(config.motor.turnSpeed);
    }
}
void TurnLeftState::onExit(ControlSubsystem* context) {
    // Prepare for next state by stopping motors
    context->getMotorA()->setSpeed(0);
    context->getMotorB()->setSpeed(0);
}

void TurnLeftState::setTurnSpeed(int speed) {
    turnSpeed = constrain(speed, 10, 80); // Limit turn speed
}

void TurnLeftState::setPIDGains(float kp, float ki, float kd) {
    pidController.setTunings(kp, ki, kd);
}
