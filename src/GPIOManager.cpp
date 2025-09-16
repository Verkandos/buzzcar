#include <Arduino.h>
#include "GPIOManager.hpp"

#ifdef ESP32
    #include "esp32-hal-ledc.h" // For PWM on ESP32
    #include "driver/ledc.h"    // For LEDC driver
#endif

GPIOManager::GPIOManager() {
    // Private constructor for singleton
}

GPIOManager& GPIOManager::getInstance() {
    static GPIOManager instance; // Thread-safe in C++11+
    return instance;
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
        // Default PWM setup (1kHz)
        configurePWMPin(pin, 1000, 8);
    }
}

void GPIOManager::configurePWMPin(int pin, int frequency, int resolution) {
    #ifdef ESP32
        // ESP32-C6 has 6 LEDC channels (0-5)
        static int channelCounter = 0;
        int channel = channelCounter % 6;  // Use channels 0-5, not pin % 16
        channelCounter++;

        ledcAttach(pin, frequency, resolution);
        ledcWrite(pin, 0);  // Initialize with 0 duty cycle
        pwmChannels[pin] = channel;

        Serial.print("PWM configured on pin: ");
        Serial.print(pin);
        Serial.print(" with frequency: ");
        Serial.print(frequency);
        Serial.print(" Hz and channel: ");
        Serial.println(channel);
    #else
        pinMode(pin, OUTPUT);
    #endif
}

// PWM Output: duty cycle 0-255
void GPIOManager::writePWM(int pin, int duty) {
    duty = constrain(duty, 0, 255);

    #ifdef ESP32
        // Use ESP32 Arduino Core 3.x function
        ledcWrite(pin, duty);  // Direct pin write for ESP32 3.x
    #else
        analogWrite(pin, duty);  // Fallback for other platforms
    #endif
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