#include <Arduino.h>
#include "UserInterface.hpp"
#include "GPIOManager.hpp"


UserInterface::UserInterface() : pinButton(USER_BUTTON_PIN), turnedOn(false) {}

UserInterface::UserInterface(int buttonPin) : pinButton(buttonPin), turnedOn(false) {}

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

    // Debouncing logic
    if (currentButtonState != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > 50) { // 50ms debounce time
        if (currentButtonState && !lastButtonState) {
            // Rising edge
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