#include "PIDController.hpp"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd),
      previousError(0.0), integral(0.0), previousTime(0),
      outputMin(-50.0), outputMax(50.0), integralLimit(25.0) {
    // Constructor initialization with outputMin/Max of +/- 50
    // means PID can adjust motor speeds by +/- 50% from base speed
}

float PIDController::compute(float setpoint, float input) {
    unsigned long currentTime = millis();

    if (previousTime == 0 ) {
        previousTime = currentTime;
        return 0.0; // No correction on first call
    }
    
    // Calculate time delta (convert to seconds)
    float deltaTime = (currentTime - previousTime) / 1000.0f; // in seconds

    // Skip computation if no time has passed (avoid division by zero)
    if (deltaTime <= 0.0) {
        return 0.0; // No time elapsed, no adjustment
    } // Compilation error resolved
    
    // Calculate error
    float error = setpoint - input;

    // Proportional term
    float proportional = kp * error;

    // Integral term (with windup prevention)
    integral += error * deltaTime;

    // Prevent integral windup
    if (integral > integralLimit) {
        integral = integralLimit;
    } else if (integral < -integralLimit) {
        integral = -integralLimit;
    }

    float integralTerm = ki * integral;

    // Derivative term
    float derivative = (error - previousError) / deltaTime;
    float derivativeTerm = kd * derivative;

    // Calculate total PID output
    float output = proportional + integralTerm + derivativeTerm;

    // Apply output limits
    if (output > outputMax) {
        output = outputMax;
    } else if (output < outputMin) {
        output = outputMin;
    }

    // Save state for next computation
    previousError = error;
    previousTime = currentTime;

    return output; // Output is the adjustment to apply
}

void PIDController::setOutputLimits(float min, float max) {
    if (min < max) { // Validate limits
        outputMin = min;
        outputMax = max;
    }
}

void PIDController::setTunings(float kp, float ki, float kd) {
    // Validate that gains are non-negative
    if (kp >= 0 && ki >= 0 && kd >= 0) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }
}

void PIDController::reset() {
    // Clear PID state - call when new line following session
    previousError = 0.0;
    integral = 0.0;
    previousTime = millis(); // Reset time reference
}

void PIDController::setIntegralLimit(float limit) {
    if (limit >= 0) { // Validate limit
        integralLimit = limit;
    }
}

void PIDController::getTunings(float& kp, float& ki, float& kd) const {
    kp = this->kp;
    ki = this->ki;
    kd = this->kd;
}