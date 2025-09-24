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
        // Use direct ESP32 LEDC driver instead of Arduino abstraction
        int channel;
        if (pin == 19) channel = 0;      // Motor B - Channel 0
        else if (pin == 20) channel = 1; // Motor A - Channel 1  
        else if (pin == 23) channel = 2; // Audio - Channel 2
        else channel = 3; // Default
        
        // Configure LEDC timer
        ledc_timer_config_t timer_config = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = (ledc_timer_bit_t)resolution,
            .timer_num = (ledc_timer_t)channel,
            .freq_hz = (uint32_t)frequency,
            .clk_cfg = LEDC_AUTO_CLK
        };
        ledc_timer_config(&timer_config);
        
        // Configure LEDC channel
        ledc_channel_config_t channel_config = {
            .gpio_num = pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)channel,
            .timer_sel = (ledc_timer_t)channel,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel_config);
        
        pwmChannels[pin] = channel;
    #endif
}

// PWM Output: duty cycle 0-255
void GPIOManager::writePWM(int pin, int duty) {
    duty = constrain(duty, 0, 255);

    #ifdef ESP32
        if (pwmChannels.find(pin) != pwmChannels.end()) {
            int channel = pwmChannels[pin];
            ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
        }
    #else
        analogWrite(pin, duty);  // Fallback
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