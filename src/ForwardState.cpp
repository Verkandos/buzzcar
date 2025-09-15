#include <Arduino.h>
#include "ForwardState.hpp"

void ForwardState::onEntry() {
    // TODO: initialize forward movement
}

void ForwardState::onUpdate() {
    // TODO: update forward movement
}

void ForwardState::onExit() {
    // TODO: cleanup forward movement
}

void ForwardState::setPIDGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

void ForwardState::setSetpoint(float target) {
    setpoint = target;
}

float ForwardState::computePID(float currentValue) {
    // TODO: Implement PID algorithm
    return 0;
}