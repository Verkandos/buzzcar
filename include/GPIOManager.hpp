#pragma once
#include <map>
#include <string>

// GPIO Pin Definitions for the project

// Photosensor Left
#define PHOTO_SENSOR_LEFT_PIN 4

// Photosensor Right
#define PHOTO_SENSOR_RIGHT_PIN 7

// Audio 
#define AUDIO_PIN 9

// LCD (TBD)
#define LCD_MOSI_PIN 10
#define LCD_SCCLK_PIN 11
#define LCD_CS_PIN 12
#define LCD_DC_RS_PIN 13
#define LCD_RST_PIN 14

// Motor A
#define MOTOR_A_PIN 15

// Motor B
#define MOTOR_B_PIN 17

// Reserved - USB-UART (Leave free for flashing/serial)
#define USB_UART_TX_PIN 20
#define USB_UART_RX_PIN 21

class GPIOManager {
    private:
        std::map<int, std::string> pinMappings;
    public:
        GPIOManager(); 

        void initializePins(const std::map<int, std::string>& mappings);
        void configurePin(int pin, const std::string& mode);
        void writePWM(int pin, bool duty);
        void writeDigital(int pin, bool value);
        int readAnalog(int pin);
        bool readDigital(int pin);
        
};