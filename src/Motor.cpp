#include "Arduino.h"
#include "Motor.hpp"
#include "GPIOManager.hpp"

Motor::Motor() 
    : pinPWM(-1), currentSpeedPercent(0), minPWM(0), maxPWM(255), minimumStartPWM(50) {
    // Default constructor
}

Motor::Motor(int pin) 
    : pinPWM(pin), currentSpeedPercent(0), minPWM(0), maxPWM(255), minimumStartPWM(50) {
    // Constructor with pin
}

void Motor::initialize() {
    if (pinPWM != -1) {
        
        GPIOManager& gpio = GPIOManager::getInstance();

        // Configure pin as PWM output with 10kHz
        gpio.configurePWMPin(pinPWM, 10000, 8);
        
        // Start with motor stopped
        applyPWM(0);
    }
}

void Motor::initialize(int pin) {
    pinPWM = pin;
    initialize();
}

void Motor::activate() {
    setSpeed(50); // Start at 50% speed when activated
}

void Motor::setSpeed(int speedPercent) {
    // speedPercent: 0-100
    currentSpeedPercent = constrain(speedPercent, 0, 100);
    
    int pwmValue;
    if (currentSpeedPercent == 0) {
        pwmValue = 0; // Complete stop
    } else {
        // Map speed percentage to effective PWM range
        // Avoid dead zone by using minimumStartPWM as lower bound
        pwmValue = map(currentSpeedPercent, 1, 100, minimumStartPWM, maxPWM);
        pwmValue = constrain(pwmValue, minimumStartPWM, maxPWM);
    }
    
    applyPWM(pwmValue);
}

void Motor::setPWMRange(int min, int max) {
    // Configure motor's operating PWM range
    minPWM = constrain(min, 0, 255);
    maxPWM = constrain(max, min, 255);
}

void Motor::setMinimumStartPWM(int pwm) {
    // Set minimum PWM needed
    // Prevents motors from stalling at low corrections
    minimumStartPWM = constrain(pwm, 0, maxPWM);
}

void Motor::stop() {
    // Immediate stop
    setSpeed(0);
}

int Motor::getCurrentSpeed() const {
    return currentSpeedPercent;
}

int Motor::getCurrentPWM() const {
    // Return actual PWM being applied
    if (currentSpeedPercent == 0) return 0;
    return map(currentSpeedPercent, 1, 100, minimumStartPWM, maxPWM);
}

bool Motor::isRunning() const {
    return currentSpeedPercent > 0;
}

int Motor::getMinimumStartPWM() const {
    return minimumStartPWM;
}

int Motor::getMaxPWM() const {
    return maxPWM;
}

// hardware interface
void Motor::applyPWM(int pwmValue) {
    if (pinPWM != -1) {
        
        GPIOManager& gpio = GPIOManager::getInstance();
        gpio.writePWM(pinPWM, pwmValue);
    }
}