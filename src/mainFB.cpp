/*
 * BuzzCar Fallback Main (mainFB.cpp)
 * 
 * Simplified line-following system WITHOUT FSM, PID, or ControlSubsystem.
 * Based on hardware_demo_main.cpp Test 9 (Full Integration Test).
 * 
 * This fallback provides:
 * - Direct sensor-to-motor control (more jerky but simpler)
 * - Button toggle on/off functionality
 * - Audio and visual feedback
 * - Continuous real-time sensor monitoring and motor response
 * 
 * 
 * Use this fallback if main.cpp has issues during final testing.
 */

#include <Arduino.h>
#include "GPIOManager.hpp"
#include "LineDetector.hpp"
#include "PhotoSensor.hpp"
#include "UserInterface.hpp"
#include "Speaker.h"
#include "Screen.h"
#include "Motor.hpp"
#include "ControlConfig.hpp"

// Global system components
GPIOManager* gpio = nullptr;
UserInterface* ui = nullptr;
PhotoSensor* sensorLeft = nullptr;
PhotoSensor* sensorCenter = nullptr;
PhotoSensor* sensorRight = nullptr;
LineDetector* lineDetector = nullptr;
Motor* motorLeft = nullptr;   // Motor A
Motor* motorRight = nullptr;  // Motor B

// System state
bool systemOn = false;

// Timing control
unsigned long lastSensorUpdate = 0;
unsigned long lastButtonCheck = 0;
unsigned long lastHeartbeat = 0;

// Configuration constants
const int SENSOR_UPDATE_INTERVAL = 100;  // Read sensors every 100ms
const int BUTTON_CHECK_INTERVAL = 50;    // Check button every 50ms
const int HEARTBEAT_INTERVAL = 3000;     // Status update every 3 seconds

void setup() {
    Serial.begin(115200);
    
    // Wait for serial connection
    while (!Serial && millis() < 2000) {
        delay(100);
    }
    delay(1000);
    
    Serial.println("========================================");
    Serial.println("BuzzCar FALLBACK System (mainFB.cpp)");
    Serial.println("Simple Line Following WITHOUT FSM/PID");
    Serial.println("========================================");
    Serial.println();
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // CRITICAL: Set pin 23 to high impedance FIRST before any other initialization
    pinMode(config.pins.protectionPin, INPUT);
    Serial.printf("Protected pin %d set to INPUT (high impedance)\n", config.pins.protectionPin);
    Serial.println();
    
    // Print configuration
    config.printConfig();
    Serial.println();
    
    // Initialize GPIO Manager
    gpio = &GPIOManager::getInstance();
    Serial.println("Initializing GPIO Manager...");
    
    // Configure all pins
    std::map<int, std::string> allPins = {
        // Input pins
        {config.pins.userButton, "digital_input_pullup"},
        {config.pins.photoSensorA, "analog_input"},
        {config.pins.photoSensorB, "analog_input"},
        {config.pins.photoSensorC, "analog_input"},
        {config.pins.protectionPin, "digital_input"}, // Pin 23 - HIGH IMPEDANCE
        
        // Output pins
        {config.pins.audio, "digital_output"},
        {config.pins.lcdData, "digital_output"},
        {config.pins.lcdClk, "digital_output"},
        {config.pins.motorA, "digital_output"},
        {config.pins.motorB, "digital_output"}
    };
    gpio->initializePins(allPins);
    Serial.println("GPIO pins configured");
    
    // Configure PWM for audio (10-bit resolution for better audio quality)
    gpio->configurePWMPin(config.pins.audio, config.feedback.audioFrequency, 10);
    Serial.printf("Audio PWM configured (Pin %d, %d Hz, 10-bit)\n", 
                  config.pins.audio, config.feedback.audioFrequency);
    
    // Configure I2C for screen
    gpio->configureI2C(config.pins.lcdData, config.pins.lcdClk);
    Serial.printf("  I2C configured for screen (SDA=%d, SCL=%d)\n", 
                  config.pins.lcdData, config.pins.lcdClk);
    
    // Configure PWM for motors (8-bit resolution)
    gpio->configurePWMPin(config.pins.motorA, config.motor.motorFrequency, 8);
    gpio->configurePWMPin(config.pins.motorB, config.motor.motorFrequency, 8);
    Serial.printf("  Motor PWM configured (Pins %d, %d @ %d Hz, 8-bit)\n", 
                  config.pins.motorA, config.pins.motorB, config.motor.motorFrequency);
    Serial.println();
    
    // Initialize UserInterface
    Serial.println("Initializing User Interface...");
    ui = new UserInterface(config.pins.userButton);
    ui->initialize();
    
    // Synchronize button state
    delay(10);
    ui->isButtonPressed();
    Serial.println("User interface ready");
    Serial.println();
    
    // Initialize Audio/Visual feedback
    Serial.println("Initializing Audio/Visual feedback...");
    speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume);
    screenBegin(config.pins.lcdData, config.pins.lcdClk);
    Serial.printf("Speaker initialized (Volume: %d%%)\n", config.feedback.audioVolume);
    Serial.println("Screen initialized");
    Serial.println();
    
    // Initialize sensors
    Serial.println("Initializing Sensors...");
    int sensorThreshold = (config.sensors.blackThreshold + config.sensors.whiteThreshold) / 2;
    sensorLeft = new PhotoSensor(config.pins.photoSensorA, sensorThreshold);
    sensorCenter = new PhotoSensor(config.pins.photoSensorB, sensorThreshold);
    sensorRight = new PhotoSensor(config.pins.photoSensorC, sensorThreshold);
    
    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();
    
    Serial.printf("PhotoSensors initialized (Pins: L=%d, C=%d, R=%d)\n",
                  config.pins.photoSensorA, config.pins.photoSensorB, config.pins.photoSensorC);
    
    // Initialize LineDetector
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight, 
                                   config.sensors.blackThreshold, config.sensors.whiteThreshold);
    Serial.printf("LineDetector initialized (Black > %d, White < %d)\n",
                  config.sensors.blackThreshold, config.sensors.whiteThreshold);
    Serial.println();
    
    // Initialize Motors
    Serial.println("Initializing Motors...");
    motorLeft = new Motor(config.pins.motorA);   // Motor A = Left wheel
    motorRight = new Motor(config.pins.motorB);  // Motor B = Right wheel
    
    motorLeft->initialize();
    motorRight->initialize();
    
    // Set minimum start PWM (using config value)
    motorLeft->setMinimumStartPWM(config.motor.minStartPWM);
    motorRight->setMinimumStartPWM(config.motor.minStartPWM);
    
    Serial.printf("Motors initialized (Left=Pin %d, Right=Pin %d)\n",
                  config.pins.motorA, config.pins.motorB);
    Serial.printf("Base speed: %d%%, Min PWM: %d\n", 
                  config.motor.baseSpeed, config.motor.minStartPWM);
    Serial.println();
    
    // Initialize screen to show STOP when system is OFF
    showDirection(0);  // Display "STOP"
    
    Serial.println("========================================");
    Serial.println("FALLBACK SYSTEM READY");
    Serial.println("========================================");
    Serial.println("Press button to START/STOP line following");
    Serial.println();
    Serial.println("System Features:");
    Serial.println("  • Direct sensor-to-motor control (no FSM)");
    Serial.println("  • Fast response: 100ms sensor updates");
    Serial.println("  • Simple(no PID)");
    Serial.println("  • Continuous audio/visual feedback");
    Serial.println();
    Serial.println("System currently: OFF - SAFE");
    Serial.println("========================================");
    Serial.println();
}

void loop() {
    ControlConfig& config = ControlConfig::getInstance();
    unsigned long currentTime = millis();
    
    // Button check every 50ms
    if (currentTime - lastButtonCheck >= BUTTON_CHECK_INTERVAL) {
        if (ui->wasButtonPressed()) {
            // Toggle system state
            systemOn = !systemOn;
            ui->toggleSystem();
            
            Serial.println("\n==========================================");
            Serial.print("BUTTON PRESSED! System now: ");
            Serial.println(systemOn ? "ACTIVE" : "STOPPED");
            Serial.println("==========================================");
            
            if (systemOn) {
                Serial.println("Line following ACTIVE - Robot will move!");
                Serial.println("Sensors read every 100ms");
                Serial.println("Direct motor control enabled");
                Serial.println("Audio/visual feedback ON");
                lastSensorUpdate = 0; // Force immediate sensor reading
            } else {
                Serial.println("Emergency STOP activated");
                Serial.println("Motors stopped immediately");
                Serial.println("Audio stopped");
                Serial.println("Screen shows STOP");
                
                // Stop everything immediately
                motorLeft->stop();
                motorRight->stop();
                speakerStop();
                showDirection(0);  // Display "STOP"
                
                Serial.printf("\n  Motor Status:\n");
                Serial.printf("    Left:  %3d%% (Running: %s)\n",
                             motorLeft->getCurrentSpeed(), 
                             motorLeft->isRunning() ? "Yes" : "No");
                Serial.printf("    Right: %3d%% (Running: %s)\n",
                             motorRight->getCurrentSpeed(),
                             motorRight->isRunning() ? "Yes" : "No");
            }
            Serial.println("==========================================\n");
        }
        
        lastButtonCheck = currentTime;
    }
    
    // Sensor reading and motor control every 100ms when ON
    if (systemOn && (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL)) {
        
        // Read raw sensor values
        int leftRaw = sensorLeft->readRaw();
        int centerRaw = sensorCenter->readRaw();
        int rightRaw = sensorRight->readRaw();
        
        // Determine sensor detections
        const char* leftDetection = (leftRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                   (leftRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* centerDetection = (centerRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                     (centerRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* rightDetection = (rightRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                    (rightRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        
        // Get line state and position from LineDetector
        LineState lineState = lineDetector->detectLineState();
        float linePosition = lineDetector->calculateLinePosition();
        
        // Map LineState to motor commands and feedback
        int leftMotorSpeed = 0;
        int rightMotorSpeed = 0;
        int direction = 0; // 0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT
        const char* action = "UNKNOWN";
        
        switch (lineState) {
            case LineState::ON_LINE:
                action = "FORWARD";
                direction = 1;
                leftMotorSpeed = config.motor.baseSpeed;
                rightMotorSpeed = config.motor.baseSpeed;
                break;
                
            case LineState::TURN_LEFT:
                action = "TURN LEFT";
                direction = 2;
                leftMotorSpeed = config.motor.turnSpeed;
                rightMotorSpeed = config.motor.baseSpeed;
                break;
                
            case LineState::TURN_RIGHT:
                action = "TURN RIGHT";
                direction = 3;
                leftMotorSpeed = config.motor.baseSpeed;
                rightMotorSpeed = config.motor.turnSpeed;
                break;
                
            case LineState::OFF_LINE:
                action = "STOP (OFF LINE)";
                direction = 0;
                leftMotorSpeed = 0;
                rightMotorSpeed = 0;
                break;
                
            case LineState::UNKNOWN:
                action = "STOP (UNKNOWN)";
                direction = 0;
                leftMotorSpeed = 0;
                rightMotorSpeed = 0;
                break;
        }
        
        // Apply motor commands
        motorLeft->setSpeed(leftMotorSpeed);
        motorRight->setSpeed(rightMotorSpeed);
        
        // Update audio/visual feedback
        startMelodyForDirection(direction);
        showDirection(direction);
        
        // Print sensor and motor status
        Serial.println("");
        Serial.printf("Sensors: L:%-5s(%4d)  C:%-5s(%4d)  R:%-5s(%4d) │\n",
                     leftDetection, leftRaw, 
                     centerDetection, centerRaw, 
                     rightDetection, rightRaw);
        Serial.printf("Line Position: %+.3f (normalized)                  │\n", linePosition);
        Serial.printf("Action: %-20s  Direction: %d          │\n", action, direction);
        Serial.printf("Motors: Left=%3d%%  Right=%3d%%                     │\n", 
                     leftMotorSpeed, rightMotorSpeed);
        Serial.println("└─────────────────────────────────────────────────────┘");
        
        lastSensorUpdate = currentTime;
    }
    
    // Heartbeat every 3 seconds when ON
    if (systemOn && (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL)) {
        Serial.println("System active - Line following in progress...");
        lastHeartbeat = currentTime;
    }
    
    // Audio service continuous when ON
    if (systemOn) {
        serviceMelody();  // Keep audio playing
    }
    
    // Small delay to prevent watchdog issues
    delay(10);
}
