#include <Arduino.h>
#include "GPIOManager.hpp"

GPIOManager::GPIOManager() {
    // Constructor 
}

void GPIOManager::initializePins(const std::map<int, std::string>& mappings) {
    // Store the pin mappings
    pinMappings = mappings;
    
    // Configure each pin based on the provided mappings
    for (const auto& mapping: pinMappings) {
        int pin = mapping.first;
        std::string type = mapping.second;
        configurePin(pin, type);
    }
}

void GPIOManager::configurePin(int pin, const std::string& mode) {
    // TODO: Configure the pin mode (INPUT, OUTPUT, etc.)
    if (mode == "digital_input") {
        pinMode(pin, INPUT);
    } else if (mode == "digital_input_pullup") {
        pinMode(pin, INPUT_PULLUP);
    } else if (mode == "analog_input") {
        // Analog pins are input by default
        pinMode(pin, INPUT);
    } else if (mode == "digital_output") {
        pinMode(pin, OUTPUT);
    } else if (mode == "pwm_output") {
        pinMode(pin, OUTPUT);
    }
}
// PWM Output: duty cycle 0-255
void GPIOManager::writePWM(int pin, int duty) {
    analogWrite(pin, constrain(duty, 0, 255));
}
// Digital Output: HIGH or LOW
void GPIOManager::writeDigital(int pin, bool value) {
    digitalWrite(pin, value ? HIGH : LOW);
}
// Analog Input (0-4095 for ESP32)
int GPIOManager::readAnalog(int pin) {
    return analogRead(pin);
}
// Digital Input: HIGH or LOW
bool GPIOManager::readDigital(int pin) {
    return digitalRead(pin) == HIGH;
}