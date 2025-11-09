#pragma once

class UserInterface {
    private:
        int pinButton;
        bool turnedOn;
        bool lastButtonState;
        unsigned long lastDebounceTime;
        
    public:
        UserInterface();
        UserInterface(int buttonPin);
        
        // Initialization
        void initialize();
        void initialize(int buttonPin);
        
        // Button Methods
        bool isButtonPressed();           // Check if button is pressed

        bool wasButtonPressed(); // Edge detection 
        void toggleSystem(); // Handle on/off logic
        bool isSystemOn() const; // Check system state
};