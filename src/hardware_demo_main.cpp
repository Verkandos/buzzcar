/*
 * BuzzCar Phase I & II Integration Tests
 * 
 * Phase I - Pairwise Integration Tests (6 tests):
 * 1. Control + UI: Button toggles system on/off with debouncing
 * 2. Control + Speaker: Direct speaker audio functionality
 * 3. Control + Screen: Direct screen display functionality
 * 4. Control + Motors: Motor class with speed control and PWM
 * 5. Control + Sensors: LineDetector with PhotoSensor integration
 * 6. Control + Power: Basic power management with LED indication
 * 
 * Phase II - Progressive Full Integration Tests (3 tests):
 * 7. UI + AV: Button controlled audio-visual feedback
 * 8. UI + AV + Sensors: Sensor-driven AV feedback
 * 9. UI + AV + Sensors + Motors: Full autonomous line-following
 * 
 * Note: All tests are standalone and do not use FSM or ControlSubsystem
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

// PHASE I: Pairwise Integration Tests - Enable ONLY ONE test at a time
const bool TEST_CONTROL_UI = false;        // Control + UI integration
const bool TEST_CONTROL_SPEAKER = false;   // Control + Speaker integration  
const bool TEST_CONTROL_SCREEN = false;    // Control + Screen integration
const bool TEST_CONTROL_AUDIOVISUAL = false; // Control + Speaker + Screen integration
const bool TEST_CONTROL_MOTORS = false;    // Control + Motors integration
const bool TEST_CONTROL_SENSORS = true;   // Control + Sensors integration
const bool TEST_CONTROL_POWER = false;     // Control + Power integration

// PHASE II: Progressive Full Integration Tests - Enable ONLY ONE test at a time
const bool TEST_UI_AV = false;           // Control + UI + Speaker + Screen (AV)
const bool TEST_UI_AV_SENSORS = false;   // Control + UI + AV + Sensors
const bool TEST_UI_AV_SENSORS_MOTORS = false; // Control + UI + AV + Sensors + Motors

// TEST 5 (SENSORS) SUB-MODE CONFIGURATION - Enable ONLY ONE mode when TEST_CONTROL_SENSORS = true
const bool SENSOR_MODE_RAW_READINGS = false;      // Show raw sensor values every 3 seconds
const bool SENSOR_MODE_LINE_DETECTION = true;    // Show LineDetection states with setup/display cycles

// Global test objects
GPIOManager* gpio = nullptr;
UserInterface* ui = nullptr;
PhotoSensor* sensorLeft = nullptr;
PhotoSensor* sensorCenter = nullptr;
PhotoSensor* sensorRight = nullptr;
LineDetector* lineDetector = nullptr;
Motor* motorA = nullptr;
Motor* motorB = nullptr;


// Test timing variables
unsigned long lastTestUpdate = 0;
unsigned long testStartTime = 0;
int testPhase = 0;
bool systemOn = false;

// Utility Functions
bool validateTestSelection() {
    int activeTests = 0;
    // Phase I tests
    if (TEST_CONTROL_UI) activeTests++;
    if (TEST_CONTROL_SPEAKER) activeTests++;
    if (TEST_CONTROL_SCREEN) activeTests++;
    if (TEST_CONTROL_AUDIOVISUAL) activeTests++;
    if (TEST_CONTROL_MOTORS) activeTests++;
    if (TEST_CONTROL_SENSORS) activeTests++;
    if (TEST_CONTROL_POWER) activeTests++;
    // Phase II tests
    if (TEST_UI_AV) activeTests++;
    if (TEST_UI_AV_SENSORS) activeTests++;
    if (TEST_UI_AV_SENSORS_MOTORS) activeTests++;
    
    if (activeTests != 1) {
        Serial.println("ERROR: Enable exactly ONE test. Current active tests: " + String(activeTests));
        return false;
    }
    return true;
}

// TEST 1: CONTROL + UI INTEGRATION

void setupControlUITest() {
    Serial.println("=== CONTROL + UI INTEGRATION TEST ===");
    Serial.println("Testing: Button toggles system on/off with debouncing");
    Serial.println("Expected: Button press toggles system state reliably");
    Serial.println();
    
    // Initialize ONLY what's needed for button testing
    gpio = &GPIOManager::getInstance();
    Serial.println("GPIO Manager initialized");
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure only the button pin
    std::map<int, std::string> buttonPin = {
        {config.pins.userButton, "digital_input_pullup"}
    };
    gpio->initializePins(buttonPin);
    
    Serial.printf("Button configured on pin %d\n", config.pins.userButton);
    
    // Initialize UserInterface
    ui = new UserInterface();
    
    testStartTime = millis();
    Serial.println("Press the user button to toggle system on/off...");
    Serial.println("System currently: OFF");
}

void runControlUITest() {
    // Check for button press every 50ms
    if (millis() - lastTestUpdate > 50) {
        
        // Check if button was pressed (with debouncing)
        if (ui->wasButtonPressed()) {
            // Toggle system state
            systemOn = !systemOn;
            ui->toggleSystem();
            
            Serial.print("Button pressed! System now: ");
            Serial.println(systemOn ? "ON" : "OFF");
            
            if (systemOn) {
                Serial.println("  -> System active - all outputs enabled");
            } else {
                Serial.println("  -> System inactive - all outputs stopped");
            }
        }

        // Show periodic status when system is on
        static unsigned long lastStatus = 0;
        if (systemOn && millis() - lastStatus > 2000) {
            Serial.println("System active - heartbeat");
            lastStatus = millis();
        }

        lastTestUpdate = millis();
    }
}

// TEST 2: CONTROL + SPEAKER INTEGRATION  

void setupControlSpeakerTest() {
    Serial.println("=== CONTROL + SPEAKER INTEGRATION TEST ===");
    Serial.println("Testing: Direct speaker audio functionality");
    Serial.println("Expected: Distinct melodies for each direction");
    Serial.println();
    
    // Initialize GPIO Manager
    gpio = &GPIOManager::getInstance();
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure audio pin
    std::map<int, std::string> speakerPins = {
        {config.pins.audio, "digital_output"}
    };
    gpio->initializePins(speakerPins);
    
    // Configure PWM for audio pin using ControlConfig values
    gpio->configurePWMPin(config.pins.audio, config.feedback.audioFrequency, 10); // From config, 10-bit resolution
    
    // Initialize speaker directly
    speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume); // Use config volume
    
    testStartTime = millis();
    testPhase = 0;
    
    Serial.println("Starting direct speaker test cycle...");
    Serial.println("Listen for: STOP, FORWARD, LEFT, RIGHT melodies");
    Serial.printf("Speaker configured on pin %d, volume %d%%\n", config.pins.audio, config.feedback.audioVolume);
}

void runControlSpeakerTest() {
    // Change audio every 3 seconds for better listening
    if (millis() - lastTestUpdate > 3000) {
        
        const char* audioNames[] = {"STOP", "FORWARD", "LEFT", "RIGHT"};
        int directions[] = {0, 1, 2, 3}; // 0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT
        
        if (testPhase < 4) {
            Serial.printf("Phase %d: Play %s melody\n", testPhase + 1, audioNames[testPhase]);
            Serial.println("  -> Listen for audio now...");
            
            // Directly call startMelodyForDirection to test speaker
            startMelodyForDirection(directions[testPhase]);
            Serial.printf("Called startMelodyForDirection(%d) for %s\n", directions[testPhase], audioNames[testPhase]);
            
            testPhase++;
        } else {
            // Reset cycle
            testPhase = 0;
            Serial.println("\n--- Audio test cycle complete, restarting ---\n");
        }
        
        lastTestUpdate = millis();
    }
    
    // Keep servicing the melody to maintain audio playback
    serviceMelody();
}

// TEST 3: CONTROL + SCREEN INTEGRATION

void setupControlScreenTest() {
    Serial.println("=== CONTROL + SCREEN INTEGRATION TEST ===");
    Serial.println("Testing: Direct screen display functionality");
    Serial.println("Expected: Screen displays each direction clearly");
    Serial.println();
    
    // Initialize GPIO Manager
    gpio = &GPIOManager::getInstance();
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure I2C pins for screen
    std::map<int, std::string> screenPins = {
        {config.pins.lcdData, "digital_output"},
        {config.pins.lcdClk, "digital_output"}
    };
    gpio->initializePins(screenPins);
    
    // Configure I2C for screen
    gpio->configureI2C(config.pins.lcdData, config.pins.lcdClk);
    
    // Initialize screen directly
    screenBegin(config.pins.lcdData, config.pins.lcdClk);
    
    testStartTime = millis();
    testPhase = 0;
    
    Serial.println("Starting direct screen test cycle...");
    Serial.println("Watch screen for: STOP, FORWARD, LEFT, RIGHT messages");
    Serial.printf("Screen I2C configured on SDA=%d, SCL=%d\n", config.pins.lcdData, config.pins.lcdClk);
}

void runControlScreenTest() {
    // Change display every 2 seconds for better visibility
    if (millis() - lastTestUpdate > 2000) {
        
        const char* displayNames[] = {"STOP", "FORWARD", "LEFT", "RIGHT"};
        int directions[] = {0, 1, 2, 3}; // 0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT
        
        if (testPhase < 4) {
            Serial.printf("Phase %d: Display %s on screen\n", testPhase + 1, displayNames[testPhase]);
            Serial.println("  -> Check screen display now...");
            
            // Directly call showDirection to test screen display
            showDirection(directions[testPhase]);
            Serial.printf("Called showDirection(%d) for %s\n", directions[testPhase], displayNames[testPhase]);
            
            testPhase++;
        } else {
            // Reset cycle
            testPhase = 0;
            Serial.println("\n--- Screen test cycle complete, restarting ---\n");
        }
        
        lastTestUpdate = millis();
    }
}

// TEST 3.5: CONTROL + AUDIOVISUAL INTEGRATION (SPEAKER + SCREEN)

void setupControlAudioVisualTest() {
    Serial.println("=== CONTROL + AUDIOVISUAL INTEGRATION TEST ===");
    Serial.println("Testing: Combined speaker and screen functionality");
    Serial.println("Expected: Synchronized audio and visual feedback for each direction");
    Serial.println();
    
    // Initialize GPIO Manager
    gpio = &GPIOManager::getInstance();
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure both audio and screen pins
    std::map<int, std::string> avPins = {
        {config.pins.audio, "digital_output"},
        {config.pins.lcdData, "digital_output"},
        {config.pins.lcdClk, "digital_output"}
    };
    gpio->initializePins(avPins);
    
    // Configure PWM for audio pin using ControlConfig values
    gpio->configurePWMPin(config.pins.audio, config.feedback.audioFrequency, 10); // From config, 8-bit resolution
    
    // Configure I2C for screen
    gpio->configureI2C(config.pins.lcdData, config.pins.lcdClk);
    
    // Initialize both speaker and screen directly
    speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume); // Use config volume
    screenBegin(config.pins.lcdData, config.pins.lcdClk);
    
    testStartTime = millis();
    testPhase = 0;
    
    Serial.println("Starting combined audiovisual test cycle...");
    Serial.println("Watch screen AND listen for: STOP, FORWARD, LEFT, RIGHT");
    Serial.printf("Speaker: Pin %d, Volume %d%% | Screen: SDA=%d, SCL=%d\n", 
                  config.pins.audio, config.feedback.audioVolume, config.pins.lcdData, config.pins.lcdClk);
}

void runControlAudioVisualTest() {
    // Change both audio and visual every 3 seconds for coordinated feedback
    if (millis() - lastTestUpdate > 3000) {
        
        const char* avNames[] = {"STOP", "FORWARD", "LEFT", "RIGHT"};
        int directions[] = {0, 1, 2, 3}; // 0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT
        
        if (testPhase < 4) {
            Serial.printf("Phase %d: %s - Audio + Visual\n", testPhase + 1, avNames[testPhase]);
            Serial.println("  -> Check screen AND listen for audio...");
            
            // Simultaneously trigger both audio and visual feedback
            startMelodyForDirection(directions[testPhase]);  // Audio feedback
            showDirection(directions[testPhase]);            // Visual feedback
            
            Serial.printf("Called startMelodyForDirection(%d) + showDirection(%d) for %s\n", 
                         directions[testPhase], directions[testPhase], avNames[testPhase]);
            
            testPhase++;
        } else {
            // Reset cycle
            testPhase = 0;
            Serial.println("\n--- AudioVisual test cycle complete, restarting ---\n");
        }
        
        lastTestUpdate = millis();
    }
    
    // Keep servicing the melody to maintain audio playback
    serviceMelody();
}

// TEST 4: CONTROL + MOTORS INTEGRATION

void setupControlMotorsTest() {
    Serial.println("=== CONTROL + MOTORS INTEGRATION TEST ===");
    Serial.println("Testing: Motor class with speed control (0%, 50%, 75%, 99%)");
    Serial.println("Expected: Motors use Motor.cpp speed mapping and PWM control");
    Serial.println();
    
    // Initialize GPIO Manager
    gpio = &GPIOManager::getInstance();
    Serial.println("GPIO Manager initialized");
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure motor pins
    std::map<int, std::string> motorPins = {
        {config.pins.motorA, "digital_output"},
        {config.pins.motorB, "digital_output"}
    };
    gpio->initializePins(motorPins);
    
    // Configure PWM for motor pins using ControlConfig values
    gpio->configurePWMPin(config.pins.motorA, config.motor.motorFrequency, 8); // 8-bit resolution (0-255)
    gpio->configurePWMPin(config.pins.motorB, config.motor.motorFrequency, 8); // 8-bit resolution (0-255)
    
    // Initialize Motor objects (uses Motor.cpp)
    motorA = new Motor(config.pins.motorA);
    motorB = new Motor(config.pins.motorB);
    
    motorA->initialize();
    motorB->initialize();
    
    // Set minimum start PWM to avoid dead zone
    motorA->setMinimumStartPWM(50);  // From Motor class
    motorB->setMinimumStartPWM(50);
    
    testStartTime = millis();
    testPhase = 0;
    
    Serial.println("Starting Motor class integration test...");
    Serial.println("Sequence: 50% -> 75% -> 99% -> STOP (3s each) -> Wait 5s -> Repeat");
    Serial.printf("Motors: A=%d, B=%d | Frequency: %dHz\n", 
                  config.pins.motorA, config.pins.motorB, config.motor.motorFrequency);
    Serial.printf("Minimum Start PWM: %d \n", motorA->getMinimumStartPWM());
    Serial.println("Both motors will run together using Motor::setSpeed()");
}

void runControlMotorsTest() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastTestUpdate;
    
    // Test cycle: 50% (3s) -> 75% (3s) -> 99% (3s) -> STOP -> Wait 5s -> Repeat  
    if (testPhase < 3) {
        // Active motor phases (0, 1, 2)
        if (elapsedTime > 3000) { // 3 seconds per phase
            int speedPercents[] = {50, 75, 99}; // Speed in percentage (0-100)
            
            Serial.printf("Phase %d: Motors at %d%% speed\n", 
                         testPhase + 1, speedPercents[testPhase]);
            
            // Use Motor class setSpeed() method
            motorA->setSpeed(speedPercents[testPhase]);
            motorB->setSpeed(speedPercents[testPhase]);
            
            Serial.printf("  -> Motor A: Speed=%d%%, PWM=%d, Running=%s\n",
                         motorA->getCurrentSpeed(), motorA->getCurrentPWM(), 
                         motorA->isRunning() ? "Yes" : "No");
            Serial.printf("  -> Motor B: Speed=%d%%, PWM=%d, Running=%s\n",
                         motorB->getCurrentSpeed(), motorB->getCurrentPWM(),
                         motorB->isRunning() ? "Yes" : "No");
            
            testPhase++;
            lastTestUpdate = currentTime;
        }
    } else if (testPhase == 3) {
        // Stop phase
        if (elapsedTime > 3000) { // 3 seconds at 99%
            Serial.println("Phase 4: Motors STOP using Motor::stop()");
            
            // Use Motor class stop() method
            motorA->stop();
            motorB->stop();
            
            Serial.printf("  -> Motor A: Speed=%d%%, Running=%s\n",
                         motorA->getCurrentSpeed(), motorA->isRunning() ? "Yes" : "No");
            Serial.printf("  -> Motor B: Speed=%d%%, Running=%s\n",
                         motorB->getCurrentSpeed(), motorB->isRunning() ? "Yes" : "No");
            
            testPhase++;
            lastTestUpdate = currentTime;
        }
    } else if (testPhase == 4) {
        // Wait phase
        if (elapsedTime > 5000) { // 5 seconds wait
            Serial.println("\n--- Motor class test cycle complete, restarting ---\n");
            testPhase = 0; // Reset cycle
            lastTestUpdate = currentTime;
        } else {
            // Show countdown during wait
            static unsigned long lastCountdown = 0;
            if (currentTime - lastCountdown > 1000) {
                int remaining = 5 - (elapsedTime / 1000);
                Serial.printf("Waiting %d seconds before next cycle...\n", remaining);
                lastCountdown = currentTime;
            }
        }
    }
}

// TEST 5: CONTROL + SENSORS INTEGRATION

void setupControlSensorsTest() {
    Serial.println("=== CONTROL + SENSORS INTEGRATION TEST ===");
    
    // Validate sensor test mode selection
    int activeModes = 0;
    if (SENSOR_MODE_RAW_READINGS) activeModes++;
    if (SENSOR_MODE_LINE_DETECTION) activeModes++;
    
    if (activeModes != 1) {
        Serial.println("ERROR: Enable exactly ONE sensor test mode!");
        Serial.printf("Current active modes: %d\n", activeModes);
        while (true) {
            delay(1000);
            Serial.println("Fix sensor mode configuration and restart!");
        }
    }
    
    if (SENSOR_MODE_RAW_READINGS) {
        Serial.println("Mode: RAW READINGS");
        Serial.println("Testing: Direct sensor readings");
        Serial.println("Expected: Raw sensor values displayed every 3 seconds");
    } else if (SENSOR_MODE_LINE_DETECTION) {
        Serial.println("Mode: LINE DETECTION STATES");
        Serial.println("Testing: LineDetector state analysis");
        Serial.println("Expected: 5s setup -> 5s state display -> repeat");
    }
    Serial.println();
    
    // Initialize GPIO Manager
    gpio = &GPIOManager::getInstance();
    Serial.println("GPIO Manager initialized");
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure sensor pins
    std::map<int, std::string> sensorPins = {
        {config.pins.photoSensorA, "analog_input"},
        {config.pins.photoSensorB, "analog_input"},
        {config.pins.photoSensorC, "analog_input"}
    };
    gpio->initializePins(sensorPins);
    
    // Initialize sensors using ControlConfig thresholds
    int sensorThreshold = (config.sensors.blackThreshold + config.sensors.whiteThreshold) / 2; // Use midpoint
    sensorLeft = new PhotoSensor(config.pins.photoSensorA, sensorThreshold);
    sensorCenter = new PhotoSensor(config.pins.photoSensorB, sensorThreshold);
    sensorRight = new PhotoSensor(config.pins.photoSensorC, sensorThreshold);
    
    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();
    
    // Initialize LineDetector using ControlConfig thresholds
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight, 
                                   config.sensors.blackThreshold, config.sensors.whiteThreshold);
    
    testStartTime = millis();
    lastTestUpdate = millis();
    testPhase = 0; // 0 = setup phase, 1 = display phase
    
    Serial.println("Sensor pins configured:");
    Serial.printf("  Left: Pin %d | Center: Pin %d | Right: Pin %d\n", 
                  config.pins.photoSensorA, config.pins.photoSensorB, config.pins.photoSensorC);
    Serial.printf("  Thresholds: Black < %d | White > %d | Detection threshold: %d\n",
                  config.sensors.blackThreshold, config.sensors.whiteThreshold, sensorThreshold);
    Serial.println();
    
    if (SENSOR_MODE_RAW_READINGS) {
        Serial.println("Starting raw sensor monitoring (3-second intervals)...");
        Serial.println("Watch the sensor values update...");
    } else if (SENSOR_MODE_LINE_DETECTION) {
        Serial.println("Starting LineDetection state monitoring (5s cycles)...");
        Serial.println("\n*** SETUP PHASE: Position your line under the sensors ***");
        Serial.println("You have 5 seconds to prepare...");
    }
}

void runControlSensorsTest() {
    if (SENSOR_MODE_RAW_READINGS) {
        // MODE 1: Raw sensor readings every 3 seconds
        if (millis() - lastTestUpdate > 3000) {
            
            // Get raw sensor readings 
            int leftRaw = sensorLeft->readRaw();
            int centerRaw = sensorCenter->readRaw();
            int rightRaw = sensorRight->readRaw();
            
            // Display raw sensor information
            Serial.println("=== RAW SENSOR READING ===");
            Serial.printf("Raw Values -> Left: %4d | Center: %4d | Right: %4d\n", 
                         leftRaw, centerRaw, rightRaw);
            Serial.printf("Threshold: %d (values below = DARK, above = LIGHT)\n", sensorLeft->getThreshold());
            Serial.println("==========================\n");
            
            lastTestUpdate = millis();
        }
        
    } else if (SENSOR_MODE_LINE_DETECTION) {
        // MODE 2: LineDetection states with 5s setup / 5s display cycles
        unsigned long elapsedTime = millis() - lastTestUpdate;
        
        if (testPhase == 0) {
            // SETUP PHASE: 5 seconds to position line
            if (elapsedTime > 5000) {
                Serial.println("\n*** DISPLAY PHASE: Showing sensor states for 5 seconds ***");
                testPhase = 1; // Move to display phase
                lastTestUpdate = millis();
            } else {
                // Show countdown during setup
                static unsigned long lastCountdown = 0;
                if (millis() - lastCountdown > 1000) {
                    int remaining = 5 - (elapsedTime / 1000);
                    Serial.printf("Setup time remaining: %d seconds...\n", remaining);
                    lastCountdown = millis();
                }
            }
            
        } else if (testPhase == 1) {
            // DISPLAY PHASE: Show sensor states every 500ms for 5 seconds
            static unsigned long lastReading = 0;
            if (millis() - lastReading > 500) {
                
                // Get raw sensor readings 
                int leftRaw = sensorLeft->readRaw();
                int centerRaw = sensorCenter->readRaw();
                int rightRaw = sensorRight->readRaw();
                
                // Get configuration instance for thresholds
                ControlConfig& config = ControlConfig::getInstance();
                
                // Determine what each sensor detects (BLACK or WHITE) [debug info]
                // const char* leftDetection = (leftRaw > config.sensors.blackThreshold) ? "BLACK" : 
                //                            (leftRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
                // const char* centerDetection = (centerRaw > config.sensors.blackThreshold) ? "BLACK" : 
                //                              (centerRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
                // const char* rightDetection = (rightRaw > config.sensors.blackThreshold) ? "BLACK" : 
                //                             (rightRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
                
                // Get LineDetector analysis
                LineState lineState = lineDetector->detectLineState();
                float linePosition = lineDetector->calculateLinePosition();
                
                // Display comprehensive sensor information
                Serial.println("=== LINE DETECTION STATE ===");
                Serial.printf("Raw Values -> Left: %4d | Center: %4d | Right: %4d\n", 
                             leftRaw, centerRaw, rightRaw);
                // Serial.printf("Detection  -> Left: %-5s | Center: %-5s | Right: %-5s\n",
                //              leftDetection, centerDetection, rightDetection);
                
                // Display line detection state
                Serial.print("Line State: ");
                switch (lineState) {
                    case LineState::ON_LINE:
                        Serial.println("ON_LINE (following straight)");
                        break;
                    case LineState::TURN_LEFT:
                        Serial.println("TURN_LEFT (line moved left)");
                        break;
                    case LineState::TURN_RIGHT:
                        Serial.println("TURN_RIGHT (line moved right)");
                        break;
                    case LineState::OFF_LINE:
                        Serial.println("OFF_LINE (no line detected)");
                        break;
                    case LineState::UNKNOWN:
                        Serial.println("UNKNOWN (ambiguous reading)");
                        break;
                }
                
                Serial.printf("Line Position: %.3f (normalized -1.0 to +1.0)\n", linePosition);
                Serial.println("============================\n");
                
                lastReading = millis();
            }
            
            // Check if display phase is complete (5 seconds)
            if (elapsedTime > 5000) {
                Serial.println("\n--- Line detection cycle complete ---");
                Serial.println("*** SETUP PHASE: Reposition your line for next test ***");
                Serial.println("You have 5 seconds to prepare...\n");
                testPhase = 0; // Return to setup phase
                lastTestUpdate = millis();
            }
        }
    }
}

// TEST 6: CONTROL + POWER INTEGRATION

void setupControlPowerTest() {
    Serial.println("=== CONTROL + POWER INTEGRATION TEST ===");
    Serial.println("Testing: Basic power management with LED indication");
    Serial.println("Expected: LED blinks to confirm ESP32 power and operation");
    Serial.println();
    
    // Configure built-in LED
    #ifdef LED_BUILTIN
        pinMode(LED_BUILTIN, OUTPUT);
        Serial.printf("Using LED_BUILTIN on pin %d\n", LED_BUILTIN);
    #else
        const int LED_PIN = 8; // ESP32-C6 typical built-in LED
        pinMode(LED_PIN, OUTPUT);
        Serial.printf("Using LED on pin %d\n", LED_PIN);
    #endif
    
    testStartTime = millis();
    testPhase = 0;
    
    Serial.println("LED will blink to indicate power and system operation...");
}

void runControlPowerTest() {
    // Blink LED every 500ms
    static bool ledState = false;
    
    if (millis() - lastTestUpdate > 500) {
        ledState = !ledState;
        
        #ifdef LED_BUILTIN
            digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
        #else
            const int LED_PIN = 8;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        #endif
        
        // Print status every 10 blinks (5 seconds)
        testPhase++;
        if (testPhase >= 10) {
            Serial.println("Power test running - LED blinking normally");
            Serial.println("System power stable, ESP32 operational");
            testPhase = 0;
        }
        
        lastTestUpdate = millis();
    }
}

// PHASE II: PROGRESSIVE FULL INTEGRATION TESTS

// TEST 7: CONTROL + UI + SPEAKER + SCREEN (AV INTEGRATION)

void setupUIAVTest() {
    Serial.println("=== PHASE II: CONTROL + UI + SPEAKER + SCREEN (AV) TEST ===");
    Serial.println("Testing: Button controls coordinated audio/visual feedback");
    Serial.println("Expected: Button toggles AV system, cycles through directions with audio + screen");
    Serial.println();
    
    // Initialize GPIO Manager
    gpio = &GPIOManager::getInstance();
    Serial.println("GPIO Manager initialized");
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure button, audio and screen pins
    std::map<int, std::string> uiAvPins = {
        {config.pins.userButton, "digital_input_pullup"},
        {config.pins.audio, "digital_output"},
        {config.pins.lcdData, "digital_output"},
        {config.pins.lcdClk, "digital_output"}
    };
    gpio->initializePins(uiAvPins);
    
    // Configure PWM for audio pin using ControlConfig values
    gpio->configurePWMPin(config.pins.audio, config.feedback.audioFrequency, 10);
    
    // Configure I2C for screen
    gpio->configureI2C(config.pins.lcdData, config.pins.lcdClk);
    
    // Initialize UserInterface
    ui = new UserInterface();
    
    // Synchronize UserInterface button state with actual pin state
    delay(10); // Allow pin to stabilize
    ui->isButtonPressed(); // This will update internal state
    
    // Initialize both speaker and screen directly
    speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume);
    screenBegin(config.pins.lcdData, config.pins.lcdClk);
    
    testStartTime = millis();
    testPhase = 0;
    systemOn = false;
    
    // Initialize screen to show STOP when system is OFF
    showDirection(0);  // Display "STOP" to indicate system is off initially
    
    Serial.println("Press button to toggle AV system on/off...");
    Serial.println("When ON: Cycles through STOP, FORWARD, LEFT, RIGHT with audio + screen");
    Serial.println("When OFF: Audio stops, screen shows 'STOP'");
    Serial.printf("Button: Pin %d | Speaker: Pin %d, Volume %d%% | Screen: SDA=%d, SCL=%d\n", 
                  config.pins.userButton, config.pins.audio, config.feedback.audioVolume, config.pins.lcdData, config.pins.lcdClk);
    Serial.println("System currently: OFF");
}

void runUIAVTest() {
    // AV cycling timing
    static unsigned long lastAVUpdate = 0;
    
    // Check for button press every 50ms
    if (millis() - lastTestUpdate > 50) {
        
        // Check if button was pressed (with debouncing)
        if (ui->wasButtonPressed()) {
            // Toggle system state
            systemOn = !systemOn;
            ui->toggleSystem();
            
            Serial.print("Button pressed! AV System now: ");
            Serial.println(systemOn ? "ON" : "OFF");
            
            if (systemOn) {
                Serial.println("  -> AV system active - cycling through directions with audio + screen");
                testPhase = 0; // Reset cycle
                lastAVUpdate = 0; // Force immediate start of AV cycle
            } else {
                Serial.println("  -> AV system inactive - audio stopped, screen shows STOP");
                
                // Stop audio and show STOP on screen immediately
                speakerStop();      // Stop any playing audio
                showDirection(0);   // Display "STOP" to indicate system is off
            }
        }
        
        // AV cycling when system is ON
        if (systemOn && millis() - lastAVUpdate > 3000) { 
            const char* avNames[] = {"STOP", "FORWARD", "LEFT", "RIGHT"};
            int directions[] = {0, 1, 2, 3}; // 0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT
            
            if (testPhase < 4) {
                Serial.printf("System ON - Phase %d: %s - Audio + Visual\n", testPhase + 1, avNames[testPhase]);
                Serial.println("  -> Check screen AND listen for audio...");
                
                // Simultaneously trigger both audio and visual feedback
                startMelodyForDirection(directions[testPhase]);  // Audio feedback
                showDirection(directions[testPhase]);            // Visual feedback
                
                testPhase++;
            } else {
                // Reset cycle
                testPhase = 0;
                Serial.println("\n--- UI + AV test cycle complete, restarting ---\n");
            }
            
            lastAVUpdate = millis();
        }
        
        // Show periodic status when system is on
        static unsigned long lastStatus = 0;
        if (systemOn && millis() - lastStatus > 2000) {
            Serial.println("AV system active - heartbeat");
            lastStatus = millis();
        }

        lastTestUpdate = millis();
    }
    
    // Keep servicing the melody to maintain audio playback
    if (systemOn) {
        serviceMelody();
    }
}

// TEST 8: CONTROL + UI + AV + SENSORS

void setupUIAVSensorsTest() {
    Serial.println("=== PHASE II: CONTROL + UI + AV + SENSORS TEST ===");
    Serial.println("Testing: Sensor-driven AV feedback with button control");
    Serial.println("Expected: Button toggles system, sensors update AV every 5 seconds");
    Serial.println();
    
    // Initialize GPIO Manager
    gpio = &GPIOManager::getInstance();
    Serial.println("GPIO Manager initialized");
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure button, audio, screen, and sensor pins
    std::map<int, std::string> uiAvSensorPins = {
        {config.pins.userButton, "digital_input_pullup"},
        {config.pins.audio, "digital_output"},
        {config.pins.lcdData, "digital_output"},
        {config.pins.lcdClk, "digital_output"},
        {config.pins.photoSensorA, "analog_input"},
        {config.pins.photoSensorB, "analog_input"},
        {config.pins.photoSensorC, "analog_input"}
    };
    gpio->initializePins(uiAvSensorPins);
    
    // Configure PWM for audio pin using ControlConfig values
    gpio->configurePWMPin(config.pins.audio, config.feedback.audioFrequency, 10);
    
    // Configure I2C for screen
    gpio->configureI2C(config.pins.lcdData, config.pins.lcdClk);
    
    // Initialize UserInterface
    ui = new UserInterface();
    
    // Synchronize UserInterface button state with actual pin state
    delay(10); // Allow pin to stabilize
    ui->isButtonPressed(); // This will update internal state
    
    // Initialize speaker and screen directly
    speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume);
    screenBegin(config.pins.lcdData, config.pins.lcdClk);
    
    // Initialize sensors using ControlConfig thresholds
    int sensorThreshold = (config.sensors.blackThreshold + config.sensors.whiteThreshold) / 2;
    sensorLeft = new PhotoSensor(config.pins.photoSensorA, sensorThreshold);
    sensorCenter = new PhotoSensor(config.pins.photoSensorB, sensorThreshold);
    sensorRight = new PhotoSensor(config.pins.photoSensorC, sensorThreshold);
    
    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();
    
    // Initialize LineDetector
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight, 
                                   config.sensors.blackThreshold, config.sensors.whiteThreshold);
    
    testStartTime = millis();
    testPhase = 0;
    systemOn = false;
    
    // Initialize screen to show STOP when system is OFF
    showDirection(0);  // Display "STOP" to indicate system is off initially
    
    Serial.println("Press button to toggle sensor-driven AV system...");
    Serial.println("When ON: Sensors update AV feedback every 5 seconds based on line position");
    Serial.println("When OFF: Audio stops, screen shows 'STOP', sensors monitor only");
    Serial.printf("Button: Pin %d | Speaker: Pin %d | Screen: SDA=%d, SCL=%d\n", 
                  config.pins.userButton, config.pins.audio, config.pins.lcdData, config.pins.lcdClk);
    Serial.printf("Sensors: L:%d C:%d R:%d (Thresholds: Black<%d, White>%d)\n",
                  config.pins.photoSensorA, config.pins.photoSensorB, config.pins.photoSensorC,
                  config.sensors.blackThreshold, config.sensors.whiteThreshold);
    Serial.println("System currently: OFF");
}

void runUIAVSensorsTest() {
    // AV update timing (5 second intervals when system is ON)
    static unsigned long lastAVUpdate = 0;
    
    // Check for button press every 50ms
    if (millis() - lastTestUpdate > 50) {
        
        // Check if button was pressed (with debouncing)
        if (ui->wasButtonPressed()) {
            // Toggle system state
            systemOn = !systemOn;
            ui->toggleSystem();
            
            Serial.print("Button pressed! Sensor-AV System now: ");
            Serial.println(systemOn ? "ON" : "OFF");
            
            if (systemOn) {
                Serial.println("  -> Sensor-driven AV active - will update based on line position every 5 seconds");
                lastAVUpdate = 0; // Force immediate sensor reading and AV update
            } else {
                Serial.println("  -> AV disabled - audio stopped, screen shows STOP");
                
                // Stop audio and show STOP on screen immediately
                speakerStop();      // Stop any playing audio
                showDirection(0);   // Display "STOP" to indicate system is off
            }
        }
        
        lastTestUpdate = millis();
    }
    
    // Sensor reading and AV feedback every 5 seconds when system is ON
    if (systemOn && millis() - lastAVUpdate > 5000) {
        
        // Get raw sensor readings
        int leftRaw = sensorLeft->readRaw();
        int centerRaw = sensorCenter->readRaw();
        int rightRaw = sensorRight->readRaw();
        
        // Get configuration instance for thresholds
        ControlConfig& config = ControlConfig::getInstance();
        
        // Determine what each sensor detects (BLACK or WHITE)
        const char* leftDetection = (leftRaw < config.sensors.blackThreshold) ? "BLACK" : 
                                   (leftRaw > config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* centerDetection = (centerRaw < config.sensors.blackThreshold) ? "BLACK" : 
                                     (centerRaw > config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* rightDetection = (rightRaw < config.sensors.blackThreshold) ? "BLACK" : 
                                    (rightRaw > config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        
        // Get LineDetector analysis
        LineState lineState = lineDetector->detectLineState();
        float linePosition = lineDetector->calculateLinePosition();
        
        // Map LineState to direction for AV feedback
        const char* stateStr = "UNKNOWN";
        const char* avStr = "None";
        int direction = 0; // 0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT
        
        switch (lineState) {
            case LineState::ON_LINE:
                stateStr = "ON_LINE";
                avStr = "Forward";
                direction = 1; // FORWARD
                break;
            case LineState::TURN_LEFT:
                stateStr = "TURN_LEFT";
                avStr = "Left Turn";
                direction = 2; // LEFT
                break;
            case LineState::TURN_RIGHT:
                stateStr = "TURN_RIGHT";
                avStr = "Right Turn";
                direction = 3; // RIGHT
                break;
            case LineState::OFF_LINE:
                stateStr = "OFF_LINE";
                avStr = "Stop";
                direction = 0; // STOP
                break;
            case LineState::UNKNOWN:
                stateStr = "UNKNOWN";
                avStr = "Stop (Unknown)";
                direction = 0; // STOP
                break;
        }
        
        // Display comprehensive sensor information
        Serial.println("\n=== SENSOR-DRIVEN AV UPDATE ===");
        Serial.printf("Raw Values -> Left: %4d | Center: %4d | Right: %4d\n", 
                     leftRaw, centerRaw, rightRaw);
        Serial.printf("Detection  -> Left: %-5s | Center: %-5s | Right: %-5s\n",
                     leftDetection, centerDetection, rightDetection);
        Serial.printf("Line State: %s\n", stateStr);
        Serial.printf("Line Position: %.3f (normalized -1.0 to +1.0)\n", linePosition);
        Serial.printf("AV Feedback: %s (direction=%d)\n", avStr, direction);
        
        // Update audio and visual feedback based on detected line state
        startMelodyForDirection(direction);  // Audio feedback
        showDirection(direction);            // Visual feedback
        
        Serial.println("  -> Screen updated & audio playing");
        Serial.println("================================\n");
        
        lastAVUpdate = millis();
    }
    
    // Show periodic heartbeat when system is on (every 2 seconds between AV updates)
    static unsigned long lastHeartbeat = 0;
    if (systemOn && millis() - lastHeartbeat > 2000) {
        Serial.println("Sensor-AV system active - monitoring line position...");
        lastHeartbeat = millis();
    }
    
    // Keep servicing the melody to maintain audio playback
    if (systemOn) {
        serviceMelody();
    }
}

// TEST 9: CONTROL + UI + AV + SENSORS + MOTORS

void setupUIAVSensorsMotorsTest() {
    Serial.println("=== PHASE II: CONTROL + UI + AV + SENSORS + MOTORS TEST ===");
    Serial.println("Testing: Complete line-following with AV feedback and motor control");
    Serial.println("Expected: Full autonomous operation with sensor-driven motors and AV");
    Serial.println();
    
    // Initialize GPIO Manager
    gpio = &GPIOManager::getInstance();
    Serial.println("GPIO Manager initialized");
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure button, audio, screen, sensor, and motor pins
    std::map<int, std::string> fullSystemPins = {
        {config.pins.userButton, "digital_input_pullup"},
        {config.pins.audio, "digital_output"},
        {config.pins.lcdData, "digital_output"},
        {config.pins.lcdClk, "digital_output"},
        {config.pins.photoSensorA, "analog_input"},
        {config.pins.photoSensorB, "analog_input"},
        {config.pins.photoSensorC, "analog_input"},
        {config.pins.motorA, "digital_output"},
        {config.pins.motorB, "digital_output"}
    };
    gpio->initializePins(fullSystemPins);
    
    // Configure PWM for audio pin using ControlConfig values
    gpio->configurePWMPin(config.pins.audio, config.feedback.audioFrequency, 10);
    
    // Configure I2C for screen
    gpio->configureI2C(config.pins.lcdData, config.pins.lcdClk);
    
    // Configure PWM for motor pins using ControlConfig values
    gpio->configurePWMPin(config.pins.motorA, config.motor.motorFrequency, 8); // 8-bit resolution (0-255)
    gpio->configurePWMPin(config.pins.motorB, config.motor.motorFrequency, 8); // 8-bit resolution (0-255)
    
    // Initialize UserInterface
    ui = new UserInterface();
    
    // Synchronize UserInterface button state with actual pin state
    delay(10); // Allow pin to stabilize
    ui->isButtonPressed(); // This will update internal state
    
    // Initialize speaker and screen directly
    speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume);
    screenBegin(config.pins.lcdData, config.pins.lcdClk);
    
    // Initialize sensors using ControlConfig thresholds
    int sensorThreshold = (config.sensors.blackThreshold + config.sensors.whiteThreshold) / 2;
    sensorLeft = new PhotoSensor(config.pins.photoSensorA, sensorThreshold);
    sensorCenter = new PhotoSensor(config.pins.photoSensorB, sensorThreshold);
    sensorRight = new PhotoSensor(config.pins.photoSensorC, sensorThreshold);
    
    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();
    
    // Initialize LineDetector
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight, 
                                   config.sensors.blackThreshold, config.sensors.whiteThreshold);
    
    // Initialize Motor objects (uses Motor.cpp)
    motorA = new Motor(config.pins.motorA);  // Left motor
    motorB = new Motor(config.pins.motorB);  // Right motor
    
    motorA->initialize();
    motorB->initialize();
    
    // Set minimum start PWM to avoid dead zone
    motorA->setMinimumStartPWM(0); // Changed to 0
    motorB->setMinimumStartPWM(0); // Changed to 0
    
    testStartTime = millis();
    testPhase = 0;
    systemOn = false;
    
    // Initialize screen to show STOP when system is OFF
    showDirection(0);  // Display "STOP" to indicate system is off initially
    
    Serial.println("FULL LINE-FOLLOWING SYSTEM READY");
    Serial.println("Press button to toggle operation...");
    Serial.println("When ON: Full line-following with motors + AV feedback every 5 seconds");
    Serial.println("When OFF: Motors stop immediately, audio stops, screen shows STOP");
    Serial.printf("Button: Pin %d | Speaker: Pin %d | Screen: SDA=%d, SCL=%d\n", 
                  config.pins.userButton, config.pins.audio, config.pins.lcdData, config.pins.lcdClk);
    Serial.printf("Sensors: L:%d C:%d R:%d (Thresholds: Black<%d, White>%d)\n",
                  config.pins.photoSensorA, config.pins.photoSensorB, config.pins.photoSensorC,
                  config.sensors.blackThreshold, config.sensors.whiteThreshold);
    Serial.printf("Motors: Left=%d Right=%d (Base Speed: %d%%)\n",
                  config.pins.motorA, config.pins.motorB, config.motor.baseSpeed);
    Serial.println("\nSystem currently: OFF - SAFE");
}

void runUIAVSensorsMotorsTest() {
    // AV and motor update timing (5 second intervals when system is ON)
    static unsigned long lastFullUpdate = 0;
    
    // Check for button press every 50ms
    if (millis() - lastTestUpdate > 50) {
        
        // Check if button was pressed (with debouncing)
        if (ui->wasButtonPressed()) {
            // Toggle system state
            systemOn = !systemOn;
            ui->toggleSystem();
            
            Serial.print("Button pressed! FULL SYSTEM now: ");
            Serial.println(systemOn ? "ACTIVE - AUTONOMOUS" : "STOPPED - SAFE");
            
            if (systemOn) {
                Serial.println("  -> Complete line-following engaged - sensors drive motors + AV");
                Serial.println("  -> WARNING: Robot will move autonomously based on line position!");
                lastFullUpdate = 0; // Force immediate sensor reading and response
            } else {
                Serial.println("  -> Emergency stop - motors stopped, audio stopped, screen shows STOP");
                
                // Stop everything immediately
                motorA->stop();
                motorB->stop();
                speakerStop();
                showDirection(0);  // Display "STOP"
                
                Serial.printf("  -> Motor A: Speed=%d%%, Running=%s\n",
                             motorA->getCurrentSpeed(), motorA->isRunning() ? "Yes" : "No");
                Serial.printf("  -> Motor B: Speed=%d%%, Running=%s\n",
                             motorB->getCurrentSpeed(), motorB->isRunning() ? "Yes" : "No");
            }
        }
        
        lastTestUpdate = millis();
    }
    
    // Full system operation (sensors -> motors + AV) every 5 seconds when system is ON
    if (systemOn && millis() - lastFullUpdate > 5000) {
        
        // Get raw sensor readings
        int leftRaw = sensorLeft->readRaw();
        int centerRaw = sensorCenter->readRaw();
        int rightRaw = sensorRight->readRaw();
        
        // Get configuration instance for thresholds
        ControlConfig& config = ControlConfig::getInstance();
        
        // Determine what each sensor detects (BLACK or WHITE)
        const char* leftDetection = (leftRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                   (leftRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* centerDetection = (centerRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                     (centerRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* rightDetection = (rightRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                    (rightRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        
        // Get LineDetector analysis
        LineState lineState = lineDetector->detectLineState();
        // LineState lineState = LineState::ON_LINE; // Hardcoded for testing
        float linePosition = lineDetector->calculateLinePosition();
        
        // Map LineState to motor commands and AV feedback
        const char* stateStr = "UNKNOWN";
        const char* avStr = "None";
        int direction = 0; // 0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT
        int leftMotorSpeed = 0;
        int rightMotorSpeed = 0;
        
        switch (lineState) {
            case LineState::ON_LINE:
                stateStr = "ON_LINE";
                avStr = "Forward";
                direction = 1; // FORWARD
                leftMotorSpeed = 50;   // Base speed 50%
                rightMotorSpeed = 50;  // Base speed 50%
                break;
            case LineState::TURN_LEFT:
                stateStr = "TURN_LEFT";
                avStr = "Left Turn";
                direction = 2; // LEFT
                leftMotorSpeed = 0;    // Stop left motor (pivot)
                rightMotorSpeed = 50;  // Right motor at 50%
                break;
            case LineState::TURN_RIGHT:
                stateStr = "TURN_RIGHT";
                avStr = "Right Turn";
                direction = 3; // RIGHT
                leftMotorSpeed = 50;   // Left motor at 50%
                rightMotorSpeed = 0;   // Stop right motor (pivot)
                break;
            case LineState::OFF_LINE:
                stateStr = "OFF_LINE";
                avStr = "Stop (Off Line)";
                direction = 0; // STOP
                leftMotorSpeed = 0;    // Stop both motors
                rightMotorSpeed = 0;
                break;
            case LineState::UNKNOWN:
                stateStr = "UNKNOWN";
                avStr = "Stop (Unknown)";
                direction = 0; // STOP
                leftMotorSpeed = 0;    // Stop both motors
                rightMotorSpeed = 0;
                break;
        }
        
        // Display comprehensive sensor and response information
        Serial.println("\n=== FULL SYSTEM UPDATE (SENSORS -> MOTORS + AV) ===");
        Serial.printf("Raw Values -> Left: %4d | Center: %4d | Right: %4d\n", 
                     leftRaw, centerRaw, rightRaw);
        Serial.printf("Detection  -> Left: %-5s | Center: %-5s | Right: %-5s\n",
                     leftDetection, centerDetection, rightDetection);
        Serial.printf("Line State: %s\n", stateStr);
        Serial.printf("Line Position: %.3f (normalized -1.0 to +1.0)\n", linePosition);
        Serial.printf("Motor Command: Left=%d%%, Right=%d%%\n", leftMotorSpeed, rightMotorSpeed);
        Serial.printf("AV Feedback: %s (direction=%d)\n", avStr, direction);
        
        // Apply motor commands using Motor class
        motorA->setSpeed(leftMotorSpeed);   // Left motor (Motor A)
        motorB->setSpeed(rightMotorSpeed);  // Right motor (Motor B)
        
        Serial.printf("  -> Motor A (Left):  Speed=%d%%, PWM=%d, Running=%s\n",
                     motorA->getCurrentSpeed(), motorA->getCurrentPWM(), 
                     motorA->isRunning() ? "Yes" : "No");
        Serial.printf("  -> Motor B (Right): Speed=%d%%, PWM=%d, Running=%s\n",
                     motorB->getCurrentSpeed(), motorB->getCurrentPWM(),
                     motorB->isRunning() ? "Yes" : "No");
        
        // Update audio and visual feedback based on detected line state
        startMelodyForDirection(direction);  // Audio feedback
        showDirection(direction);            // Visual feedback
        
        Serial.println("  -> Screen updated & audio playing");
        Serial.println("====================================================\n");
        
        lastFullUpdate = millis();
    }
    
    // Show periodic heartbeat when system is on (every 2 seconds between full updates)
    static unsigned long lastHeartbeat = 0;
    if (systemOn && millis() - lastHeartbeat > 2000) {
        Serial.println("Full system active - autonomous line-following in progress...");
        lastHeartbeat = millis();
    }
    
    // Keep servicing the melody to maintain audio playback
    if (systemOn) {
        serviceMelody();
    }
}

// MAIN SETUP AND LOOP

void setup() {
    Serial.begin(115200);
    
    // Wait for serial connection
    while (!Serial && millis() < 5000) {
        delay(100);
    }
    delay(1000);
    
    Serial.println("BuzzCar Integration Tests - Phase I & II");
    Serial.println("=======================================");
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // CRITICAL: Set pin 23 to high impedance
    pinMode(config.pins.protectionPin, INPUT);
    Serial.printf("Protected pin %d set to INPUT (high impedance)\n", config.pins.protectionPin);
    Serial.println();
    
    // Validate test selection
    if (!validateTestSelection()) {
        while (true) {
            delay(1000);
            Serial.println("Fix test configuration and restart!");
        }
    }
    
    // Set CPU frequency for consistent timing
    setCpuFrequencyMhz(160);
    Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
    Serial.println();
    
    // Run appropriate test setup
    // Phase I: Pairwise Integration Tests
    if (TEST_CONTROL_UI) {
        setupControlUITest();
    } else if (TEST_CONTROL_SPEAKER) {
        setupControlSpeakerTest();
    } else if (TEST_CONTROL_SCREEN) {
        setupControlScreenTest();
    } else if (TEST_CONTROL_AUDIOVISUAL) {
        setupControlAudioVisualTest();
    } else if (TEST_CONTROL_MOTORS) {
        setupControlMotorsTest();
    } else if (TEST_CONTROL_SENSORS) {
        setupControlSensorsTest();
    } else if (TEST_CONTROL_POWER) {
        setupControlPowerTest();
    }
    // Phase II: Progressive Full Integration Tests
    else if (TEST_UI_AV) {
        setupUIAVTest();
    } else if (TEST_UI_AV_SENSORS) {
        setupUIAVSensorsTest();
    } else if (TEST_UI_AV_SENSORS_MOTORS) {
        setupUIAVSensorsMotorsTest();
    }
    
    Serial.println("Test initialized. Starting execution...\n");
}

void loop() {
    // Run appropriate test loop
    // Phase I: Pairwise Integration Tests
    if (TEST_CONTROL_UI) {
        runControlUITest();
    } else if (TEST_CONTROL_SPEAKER) {
        runControlSpeakerTest();
    } else if (TEST_CONTROL_SCREEN) {
        runControlScreenTest();
    } else if (TEST_CONTROL_AUDIOVISUAL) {
        runControlAudioVisualTest();
    } else if (TEST_CONTROL_MOTORS) {
        runControlMotorsTest();
    } else if (TEST_CONTROL_SENSORS) {
        runControlSensorsTest();
    } else if (TEST_CONTROL_POWER) {
        runControlPowerTest();
    }
    // Phase II: Progressive Full Integration Tests
    else if (TEST_UI_AV) {
        runUIAVTest();
    } else if (TEST_UI_AV_SENSORS) {
        runUIAVSensorsTest();
    } else if (TEST_UI_AV_SENSORS_MOTORS) {
        runUIAVSensorsMotorsTest();
    }
    
    delay(10); // Small delay to prevent watchdog issues
}
