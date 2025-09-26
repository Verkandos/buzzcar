#include <Arduino.h>
#include "TurnRightState.hpp"
#include "ControlSubsystem.hpp"
#include "Event.hpp"
#include "FSM.hpp"

TurnRightState::TurnRightState() : baseSpeed(50), turnSpeed(30), pidController(1.5f, 0.05f, 0.8f) {
    // Initialize with conservative turn speeds and PID gains
    pidController.setOutputLimits(-15.0f, 15.0f); // smaller corrections during turns
}

void TurnRightState::onEntry(ControlSubsystem* context) {
    // Initialize with conservative turn speeds and PID gains
    pidController.reset();

    // Start turning right: left motor forward, right motor stopped
    context->getMotorA()->setSpeed(turnSpeed); // Left motor at turn speed
    context->getMotorB()->setSpeed(0); // Right motor stopped

}

void TurnRightState::onUpdate(ControlSubsystem* context) {
    // Check line state to see if turn is complete
    LineState currentState = context->getLineDetector()->detectLineState();


    switch (currentState) {
        case LineState::ON_LINE:
            // Turn complete - center sensor back on line
            context->getFSM()->handleEvent(Event(EventType::FORWARD));
            break;
        case LineState::TURN_RIGHT:
            // Still need to turn - continue with PID fine-tuning
            {
                float linePosition = context->getLineDetector()->calculateLinePosition();
                float pidOutput = pidController.compute(0.5f, linePosition); // Target slightly right of center

                // Adjust motor speeds based on PID output
                int leftMotorSpeed = turnSpeed + (int)pidOutput;
                leftMotorSpeed = constrain(leftMotorSpeed, 10, 80); // Keep within safe range

                context->getMotorA()->setSpeed(leftMotorSpeed); // Adjusted left motor
                context->getMotorB()->setSpeed(0); // Right motor remains still off
            }
            break;

        case LineState::OFF_LINE:
            // Lost the line - stop and wait for further instructions
            context->getFSM()->handleEvent(Event(EventType::STOP));
            break;

        case LineState::TURN_LEFT:
            // Shouldn't happen during a right turn, but if it does, go to forward
            context->getFSM()->handleEvent(Event(EventType::FORWARD));
            break;
        
        default:
            // Unknown state - continue turning cautiously
            context->getMotorA()->setSpeed(turnSpeed / 2); // Slow turn
            context->getMotorB()->setSpeed(0);
            break;
        }
}

void TurnRightState::onExit(ControlSubsystem* context) {
    // Stop motors when exiting turn state
    context->getMotorA()->setSpeed(0);
    context->getMotorB()->setSpeed(0);
}

void TurnRightState::setTurnSpeed(int speed) {
    turnSpeed = constrain(speed, 10, 80); // Limit turn speed
}

void TurnRightState::setPIDGains(float p, float i, float d) {
    pidController.setTunings(p, i, d);
}