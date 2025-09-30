#include <Arduino.h>
#include "GPIOManager.hpp"

#ifdef ESP32
    #include "esp32-hal-ledc.h" // For PWM on ESP32
    #include "driver/ledc.h"    // For LEDC driver
#endif
/**
 * @brief Private constructor for singleton pattern
 * 
 * Initializes the GPIOManager instance. Constructor is private to ensure
 * that only one instance of the class can be created.
 */
GPIOManager::GPIOManager() {
    // Private constructor for singleton
}

/**
 * @brief Gets the singleton instance of GPIOManager
 *
 * Thread-safe singleton implementation that returns the single instance
 * of GPIOManager. Creates the instance on the first call.
 * 
 * @return GPIOManager& Reference to the singleton instance
 */
GPIOManager& GPIOManager::getInstance() {
    static GPIOManager instance; // Thread-safe in C++11+
    return instance;
}

/**
 * @brief Initializes multiple pins with their respective configurations
 * 
 * Configures multiple GPIO pins at once based on the provided mappings.
 * Stores the pin mappings internally and calls configurePin for each pin.
 * 
 * @param mappings A map of pin numbers to their configuration types
 *               ("digital_input", "digital_input_pullup", "analog_input",
 *               "analog_input", "digital_output", "pwm_output")
 */
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

/**
 * @brief Configures a single GPIO pin based on the specified mode
 * 
 * Sets up a GPIO pin according to the specified mode.
 * For PWM output, it automatically calls configurePWMPin with default parameters.
 * 
 * @param pin The GPIO pin number to configure
 * @param mode The configuration mode ("digital_input", "digital_input_pullup",
 *            "analog_input", "digital_output", "pwm_output")
 */
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


/**
 * @brief Configures a GPIO pin for PWM output with specified parameters
 * 
 * Sets up PWM output using ESP32's LEDC (LED Controller) peripheral.
 * Assigns LEDC channels based on predefined pin mappings for motors and audio
 * 
 * @param pin The GPIO pin number to configure for PWM
 * @param frequency The PWM frequency in Hz (default: 1000)
 * @param resolution The PWM resolution in bits (8= 0-255 range, 10-bit = 0-1023)
 * 
 * @note Pin-to-channel mapping:
 *      - Pin 19: Channel 0 (Motor B)
 *     - Pin 20: Channel 1 (Motor A)
 *     - Pin 23: Channel 2 (Audio)
 *    - Other pins: Channel 3 (Default)
 */
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

/**
 * @brief Writes a PWM duty cycle to a configured PWM pin
 * 
 * Sets the PWM duty cycle for the specified pin. The duty cycle is
 * constrained to the range 0-255 and applied using ESP32's LEDC.
 * 
 * @param pin The GPIO pin number to write PWM to
 * @param duty The PWM duty cycle (0-255)
 * 
 * @note Pin must be configured with configurePWMPin() 
 * first before calling this function.
 *
 */
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
/**
 * @brief Writes a digital vlaue to an outpin pin
 * 
 * Sets the digital output pin to HIGH or LOW.
 * Pin must be configured as digital output before calling this function.
 * 
 * @param pin The GPIO pin number to write to
 * @param value The digital value to write (true=HIGH, false=LOW)
 */
void GPIOManager::writeDigital(int pin, bool value) {
    digitalWrite(pin, value ? HIGH : LOW);
}
/**
 * @brief Reads an analog value from an input pin
 * 
 * Reads the analog value on the specified pin and returns the digital
 * value. ESP32 ADC returns values from 0-4095 (12-bit).
 * 
 * @param pin The GPIO pin number to read from
 * @return int The analog value read from the pin (0-4095)
 */
int GPIOManager::readAnalog(int pin) {
    return analogRead(pin);
}
/**
 * @brief Reads a digital value from an input pin
 * 
 * Reads the digital value on the specified pin and returns true for HIGH
 * 
 * @param pin The GPIO pin number to read from
 * @return bool True if the pin reads HIGH, false if LOW
 */
bool GPIOManager::readDigital(int pin) {
    return digitalRead(pin) == HIGH;
}