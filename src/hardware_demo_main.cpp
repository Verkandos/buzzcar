/*
 * BuzzCar Simple Hardware Demo
 * 
 * Simple hardware test for integration:
 * 1. Shows real-time sensor readings from all 3 photosensors
 * 2. Tests both motors (OFF and 50% speed)
 * 3. 9600 baud serial output
 * 4. 160MHz CPU frequency
 */

#include <Arduino.h>
#include "GPIOManager.hpp"
#include "Motor.hpp"
#include "PhotoSensor.hpp"

// Hardware components  
Motor motorA(MOTOR_A_PIN);
Motor motorB(MOTOR_B_PIN);
PhotoSensor sensorA(PHOTO_SENSOR_A_PIN);
PhotoSensor sensorB(PHOTO_SENSOR_B_PIN);
PhotoSensor sensorC(PHOTO_SENSOR_C_PIN);

// Simple test state
bool motorsOn = false;
unsigned long lastToggle = 0;
const unsigned long TOGGLE_INTERVAL = 3000; // 3 seconds

void setup() {
    // Set CPU frequency to 160MHz
    setCpuFrequencyMhz(160);
    
    // Initialize serial at 9600 baud
    Serial.begin(9600); // ISSUE: Serial not working on ESP32-C6
    delay(2000); // Give serial time to initialize
    
    Serial.println();
    Serial.println("=== BuzzCar Hardware Demo Starting ===");
    Serial.println("BuzzCar Hardware Demo");
    Serial.println("====================");
    Serial.println("Serial communication working!");
    
    // Check CPU frequency
    Serial.print("CPU Frequency: ");
    Serial.print(getCpuFrequencyMhz());
    Serial.println(" MHz");
    
    // Initialize hardware
    Serial.println("Initializing hardware...");
    
    // Motors
    motorA.initialize();
    motorB.initialize();
    Serial.println("Motors initialized");
    
    // Sensors
    sensorA.initialize();
    sensorB.initialize(); 
    sensorC.initialize();
    Serial.println("Sensors initialized");
    
    Serial.println("Starting demo...");
    Serial.println();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Toggle motors every 3 seconds
    if (currentTime - lastToggle >= TOGGLE_INTERVAL) {
        motorsOn = !motorsOn;
        lastToggle = currentTime;
        
        if (motorsOn) {
            Serial.println("Motors: ON (50% speed)");
            motorA.setSpeed(50);
            motorB.setSpeed(50);
        } else {
            Serial.println("Motors: OFF");
            motorA.stop();
            motorB.stop();
        }
        Serial.println();
    }
    
    // Read and display sensors
    int rawA = sensorA.readRaw();
    int rawB = sensorB.readRaw();
    int rawC = sensorC.readRaw();
    
    Serial.print("Sensors: A=");
    Serial.print(rawA);
    Serial.print("  B=");
    Serial.print(rawB);
    Serial.print("  C=");
    Serial.print(rawC);
    
    // Show motor status
    Serial.print("  |  Motors: ");
    Serial.print(motorsOn ? "50%" : "OFF");
    
    Serial.println();
    
    delay(500); // Update every 500ms
}