#include <Arduino.h>
#include "UserInterface.hpp"
#include "GPIOManager.hpp"


UserInterface::UserInterface() : pinButton(USER_BUTTON_PIN), turnedOn(false), lastButtonState(true), lastDebounceTime(0) {
    // Initialize lastButtonState to HIGH (unpressed state with pullup resistor)
}

UserInterface::UserInterface(int buttonPin) : pinButton(buttonPin), turnedOn(false), lastButtonState(true), lastDebounceTime(0) {
    // Initialize lastButtonState to HIGH (unpressed state with pullup resistor)
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
    return gpio.readDigital(pinButton);
}

bool UserInterface::wasButtonPressed() {
    if (pinButton == -1) {
        return false; // Button pin not initialized
    }

    GPIOManager& gpio = GPIOManager::getInstance();
    bool currentButtonState = gpio.readDigital(pinButton);
    bool pressed = false;

    // Debug: Show button state periodically (less spam)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 2000) { // Debug every 2 seconds
        Serial.printf("DEBUG: Button pin %d state: %s (last: %s)\n", 
                     pinButton, 
                     currentButtonState ? "HIGH" : "LOW",
                     lastButtonState ? "HIGH" : "LOW");
        lastDebugTime = millis();
    }

    // Debouncing logic
    if (currentButtonState != lastButtonState) {
        Serial.printf("DEBUG: Button state changed from %s to %s\n", 
                     lastButtonState ? "HIGH" : "LOW", 
                     currentButtonState ? "HIGH" : "LOW");
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > 50) { // 50ms debounce time
        if (!currentButtonState && lastButtonState) {
            // Falling edge (HIGH to LOW) - for pullup resistor configuration
            Serial.println("DEBUG: Button press detected (falling edge)!");
            pressed = true; // Button was pressed
        }
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