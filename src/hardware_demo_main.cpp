/*
 * BuzzCar Static PWM Signal Test
 * 
 * Static PWM output test:
 * - Motor A & B at 10kHz/50% duty cycle (continuous)
 * - Speaker at 5kHz/50% duty cycle (continuous)
 * - 115200 baud serial output
 * - 160MHz CPU frequency
 */

#include <Arduino.h>
#include "GPIOManager.hpp"

// PWM test parameters
const int MOTOR_FREQUENCY = 10000; // 10kHz for motors
const int AUDIO_FREQUENCY = 5000;  // 5kHz for speaker
const int DUTY_CYCLE_50_PERCENT = 128; // 50% of 255 (8-bit resolution)

void setup() {
    // Initialize serial at 115200 baud
    #if ARDUINO_USB_CDC_ON_BOOT
        Serial.begin(); // No baud rate needed for USB CDC
    #else
        Serial.begin(115200);
    #endif
    delay(2000); // Give serial time to initialize

    // Set CPU frequency to 160MHz
    setCpuFrequencyMhz(160);
    
    // Check CPU frequency
    Serial.print("CPU Frequency: ");
    Serial.print(getCpuFrequencyMhz());
    Serial.println(" MHz");
    
    // Get GPIO Manager instance
    GPIOManager& gpio = GPIOManager::getInstance();
    
    // Configure and start PWM pins
    Serial.println("Configuring PWM pins...");
    
    gpio.configurePWMPin(MOTOR_A_PIN, MOTOR_FREQUENCY, 8);  // 10kHz
    gpio.configurePWMPin(MOTOR_B_PIN, MOTOR_FREQUENCY, 8);  // 10kHz
    gpio.configurePWMPin(AUDIO_PIN, AUDIO_FREQUENCY, 8);    // 5kHz
    
    Serial.println("Starting PWM signals at 50% duty cycle...");
    
    // Set static 50% duty cycle on all pins
    gpio.writePWM(MOTOR_A_PIN, DUTY_CYCLE_50_PERCENT);  // Motor A
    gpio.writePWM(MOTOR_B_PIN, DUTY_CYCLE_50_PERCENT);  // Motor B
    gpio.writePWM(AUDIO_PIN, DUTY_CYCLE_50_PERCENT);    // Audio
    
}

void loop() {
    // Just keep running - PWM signals stay active
    delay(5000);
}