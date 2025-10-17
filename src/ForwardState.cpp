#include <Arduino.h>
#include "ForwardState.hpp"
#include "ControlSubsystem.hpp"
#include "ControlConfig.hpp"

ForwardState::ForwardState() : State("ForwardState"), baseSpeed(50), pidController(2.0f, 0.1f, 1.0f) {
    // Initialize with 50% base speed and reasonable PID gains
    pidController.setOutputLimits(-25.0f, 25.0f); // Limit PID output to +/- 50
}

void ForwardState::onEntry(ControlSubsystem* context) {
    ControlConfig& config = ControlConfig::getInstance();
    // Reset PID controller for new line following session
    pidController.reset();

    // Start both motors at base speed
    context->getMotorA()->setSpeed(config.motor.baseSpeed); // Left motor
    context->getMotorB()->setSpeed(config.motor.baseSpeed); // Right motor

}

void ForwardState::onUpdate(ControlSubsystem* context) {
    // Get line position from LineDetector (0.0 = centered, negative = line to left, positive = line to right)
    float linePosition = context->getLineDetector()->calculateLinePosition();

    // PID setpoint is 0.0 (we want to stay centered on the line)
    float pidOutput = pidController.compute(0.0f, linePosition);

    // Apply differential steering correction
    // If line is to the left (negative), we need to turn left
    int leftMotorSpeed = baseSpeed - (int)pidOutput;
    int rightMotorSpeed = baseSpeed + (int)pidOutput;

    // Ensure motor speeds stay within PWM range (0-100%)
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 100);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 100);

    // Apply corrected speeds to motors
    context->getMotorA()->setSpeed(leftMotorSpeed);
    context->getMotorB()->setSpeed(rightMotorSpeed);

}

void ForwardState::onExit(ControlSubsystem* context) {
    // Stop both motors when exiting forward state
    context->getMotorA()->setSpeed(0);
    context->getMotorB()->setSpeed(0);
}

void ForwardState::setPIDGains(float p, float i, float d) {
    pidController.setTunings(p, i, d);
}

void ForwardState::setBaseSpeed(int speed) {
    baseSpeed = constrain(speed, 0, 100);
}