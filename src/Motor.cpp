#include "Arduino.h"
#include "Motor.hpp"

Motor::Motor() 
    : pinPWM(-1), currentPWM(0) {
        // Default constructor
}

void Motor::activate() {
    // TODO: Activate the motor with current PWM value
}

void Motor::setPWM(int pwm) {
    // TODO: Set the PWM value for the motor
}
int Motor::getPWM() const {
    return currentPWM;
}
void Motor::stop() {
    // TODO: Stop motor
}



