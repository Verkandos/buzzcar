#pragma once
#include <map>
#include <string>

// GPIO Pin Definitions for the project

// Photosensor A (Analog)
#define PHOTO_SENSOR_A_PIN 15

// Photosensor B (Analog)
#define PHOTO_SENSOR_B_PIN 23

// Photosensor C (Analog)
#define PHOTO_SENSOR_C_PIN 22

// Audio (Output PWM)
#define AUDIO_PIN 5

// LCD (TBD)
#define LCD_MOSI_PIN 6
#define LCD_SCCLK_PIN 7
#define LCD_CS_PIN 10
#define LCD_DC_RS_PIN 11
#define LCD_RST_PIN 2

// Motor A (PWM Output)
#define MOTOR_A_PIN 21

// Motor B
#define MOTOR_B_PIN 20


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