#pragma once
#include <map>
#include <string>

// GPIO Pin Definitions for the project
// ===========INPUTS===========

// Photosensor A (Input Analog: ADC Pin)
#define PHOTO_SENSOR_A_PIN 3 // [X]

// Photosensor B (Input Analog: ADC Pin)
#define PHOTO_SENSOR_B_PIN 2 // [X]

// Photosensor C (Input Analog: ADC Pin)
#define PHOTO_SENSOR_C_PIN 1 // [X]

// User Button (Input Digital: GPIO Pin)
#define USER_BUTTON_PIN 11 // [X]

// ===========OUTPUTS==========

// Audio (Output [PWM]: GPIO Pin)
#define AUDIO_PIN 23 // [X]

// LCD (GPIO Pin)
#define LCD_DATA_PIN 22 // [X]
#define LCD_CLK_PIN 21 // [X]


// Motor A (Output [PWM]: GPIO Pin)
#define MOTOR_A_PIN 20 // [X]

// Motor B (Output [PWM]: GPIO Pin)
#define MOTOR_B_PIN 19 // [X]



class GPIOManager {
    private:
        std::map<int, std::string> pinMappings;
        std::map<int, int> pwmChannels;
        
        // Singleton pattern - private constructor and members
        GPIOManager(); 
        
        // Prevent copying
        GPIOManager(const GPIOManager&) = delete;
        GPIOManager& operator=(const GPIOManager&) = delete;
        
    public:
        // Static method to get the singleton instance
        static GPIOManager& getInstance();

        void initializePins(const std::map<int, std::string>& mappings);
        void configurePin(int pin, const std::string& mode);
        void configurePWMPin(int pin, int frequency = 1000, int resolution = 8);
        void writePWM(int pin, int duty);
        void writeDigital(int pin, bool value);
        int readAnalog(int pin);
        bool readDigital(int pin);
        
};