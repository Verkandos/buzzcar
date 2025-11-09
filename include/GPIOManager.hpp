#pragma once
#include <map>
#include <string>

// GPIO Pin Definitions moved to ControlConfig.hpp

class GPIOManager {
    private:
        std::map<int, std::string> pinMappings;
        std::map<int, int> pwmChannels;

        int i2cSDAPin = -1;
        int i2cSCLPin = -1;
        
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
        int readAnalog(int pin);
        bool readDigital(int pin);
        void setFrequency(int pin, int frequency);
        void configureI2C(int sdaPin, int sclPin, uint32_t frequency = 100000);
        
};