#include <Arduino.h>
#include "UserInterface.hpp"
#include "GPIOManager.hpp"
#include "ControlConfig.hpp"

UserInterface::UserInterface() : pinButton(ControlConfig::getInstance().pins.userButton), turnedOn(false), lastButtonState(true), lastDebounceTime(0) {
    // Initialize lastButtonState to HIGH (unpressed state at ~3.3V)
}

UserInterface::UserInterface(int buttonPin) : pinButton(buttonPin), turnedOn(false), lastButtonState(true), lastDebounceTime(0) {
    // Initialize lastButtonState to HIGH (unpressed state at ~3.3V)
}

void UserInterface::initialize() {
    if (pinButton != -1) {
        GPIOManager& gpio = GPIOManager::getInstance();
        gpio.configurePin(pinButton, "INPUT");
    }
}

void UserInterface::initialize(int buttonPin) {
    pinButton = buttonPin;
    initialize();
}
bool UserInterface::isButtonPressed() {
    if (pinButton == -1) {
        return false; // Button pin not initialized
    }
    GPIOManager& gpio = GPIOManager::getInstance();
    // Button pressed = ~0V (reads as LOW), unpressed = ~3.3V (reads as HIGH)
    return !gpio.readDigital(pinButton); // Active LOW: LOW = pressed
}

bool UserInterface::wasButtonPressed() {
    if (pinButton == -1) {
        return false; // Button pin not initialized
    }

    GPIOManager& gpio = GPIOManager::getInstance();
    bool currentButtonState = gpio.readDigital(pinButton);
    bool pressed = false;
    
    unsigned long currentTime = millis();
    unsigned long timeSinceLastChange = currentTime - lastDebounceTime;

    // Debouncing logic
    if (currentButtonState != lastButtonState) {
        lastDebounceTime = currentTime;
        Serial.printf("DEBUG_UI: State changed from %s to %s, debounce timer reset\n", 
                     lastButtonState ? "HIGH" : "LOW", currentButtonState ? "HIGH" : "LOW");
    }

    if (timeSinceLastChange > 50) { // 50ms debounce time
        if (currentButtonState == LOW && lastButtonState == HIGH) {
            // Falling edge (HIGH to LOW) - button goes from ~3.3V to ~0V when pressed
            pressed = true; // Button was pressed
            Serial.printf("DEBUG_UI: BUTTON PRESS DETECTED! timeSince=%lu\n", timeSinceLastChange);
        }
    } else {
        Serial.printf("DEBUG_UI: Still debouncing - timeSince=%lu (need >50)\n", timeSinceLastChange);
    }

    lastButtonState = currentButtonState;
    return pressed;
}

void UserInterface::toggleSystem() {
    turnedOn = !turnedOn;
}

bool UserInterface::isSystemOn() const {
    return turnedOn;
}