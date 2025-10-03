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
    // Check line state to see if turn is complete
    LineState currentState = context->getLineDetector()->detectLineState();

    switch (currentState) {
        case LineState::ON_LINE:
            // If back on line, transition to Forward state
            context->getFSM()->handleEvent(Event(EventType::FORWARD));
            break;
        case LineState::TURN_LEFT:
            // Still need to turn - continue current motor configuration
            // Use PID to fine-turn the turn based on how far off we are
            {
                float linePosition = context->getLineDetector()->calculateLinePosition();
                float pidOutput = pidController.compute(-0.5f, linePosition); // Target is -0.5 for left turn

                // Adjust motor speeds based on PID output
                int rightMotorSpeed = turnSpeed + (int)pidOutput;
                rightMotorSpeed = constrain(rightMotorSpeed, 10, 80); // Ensure within bounds

                context->getMotorA()->setSpeed(0);
                context->getMotorB()->setSpeed(rightMotorSpeed); // Adjusted right motor
            }
            break;

        case LineState::OFF_LINE:
            // Lost the line - stop and wait for further instructions
            context->getFSM()->handleEvent(Event(EventType::STOP));
            break;
        case LineState::TURN_RIGHT:
            // Shouldn't happen during a left turn, but if it does, go to forward
            context->getFSM()->handleEvent(Event(EventType::FORWARD));
            break;
        default:
            // Unknown state - continue turning cautiously
            context->getMotorA()->setSpeed(0);
            context->getMotorB()->setSpeed(turnSpeed / 2); // Slow turn
            break;
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
