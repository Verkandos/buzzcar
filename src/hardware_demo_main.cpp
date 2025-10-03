/*
 * BuzzCar Phase I Pairwise Integration Tests
 * 
 * Clean implementation of 6 integration test pairs:
 * 1. Control + UI: Button toggles system on/off with debouncing
 * 2. Control + Speaker: FSM state transitions with audio feedback
 * 3. Control + Screen: FSM state transitions with visual feedback
 * 4. Control + Motors: Motor control with PWM and differential steering
 * 5. Control + Sensors: LineDetector with PhotoSensor integration
 * 6. Control + Power: Basic power management with LED indication
 */

#include <Arduino.h>
#include "GPIOManager.hpp"
#include "LineDetector.hpp"
#include "PhotoSensor.hpp"
#include "ControlSubsystem.hpp"
#include "UserInterface.hpp"
#include "FSM.hpp"
#include "Speaker.h"
#include "Screen.h"
#include "ControlConfig.hpp"

// PHASE I: Pairwise Integration Tests - Enable ONLY ONE test at a time
const bool TEST_CONTROL_UI = false;      // Control + UI integration
const bool TEST_CONTROL_SPEAKER = false; // Control + Speaker integration  
const bool TEST_CONTROL_SCREEN = false;  // Control + Screen integration
const bool TEST_CONTROL_MOTORS = false;  // Control + Motors integration
const bool TEST_CONTROL_SENSORS = false; // Control + Sensors integration
const bool TEST_CONTROL_POWER = false;   // Control + Power integration

// PHASE II: Progressive Full Integration Tests - Enable ONLY ONE test at a time
const bool TEST_UI_AV = false;           // Control + UI + Speaker + Screen (AV)
const bool TEST_UI_AV_SENSORS = false;   // Control + UI + AV + Sensors
const bool TEST_UI_AV_SENSORS_MOTORS = false; // Control + UI + AV + Sensors + Motors

// Global test objects
GPIOManager* gpio = nullptr;
ControlSubsystem* controlSystem = nullptr;
UserInterface* ui = nullptr;
FSM* fsm = nullptr;
PhotoSensor* sensorLeft = nullptr;
PhotoSensor* sensorCenter = nullptr;
PhotoSensor* sensorRight = nullptr;
LineDetector* lineDetector = nullptr;

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

void initializeCommonComponents() {
    gpio = &GPIOManager::getInstance();
    Serial.println("GPIO Manager initialized");
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure common hardware pins that multiple tests might use
    std::map<int, std::string> commonPins = {
        // Input pins
        {PHOTO_SENSOR_A_PIN, "analog_input"},  // Pin 3 - Left sensor
        {PHOTO_SENSOR_B_PIN, "analog_input"},  // Pin 2 - Center sensor  
        {PHOTO_SENSOR_C_PIN, "analog_input"},  // Pin 1 - Right sensor
        {USER_BUTTON_PIN, "digital_input_pullup"}, // Pin 11 - User button
        
        // Output pins  
        {MOTOR_A_PIN, "digital_output"},       // Pin 20 - Motor A
        {MOTOR_B_PIN, "digital_output"},       // Pin 19 - Motor B
        {AUDIO_PIN, "digital_output"},         // Pin 23 - Speaker
        {LCD_DATA_PIN, "digital_output"},      // Pin 22 - Screen SDA
        {LCD_CLK_PIN, "digital_output"}        // Pin 21 - Screen SCL
    };
    
    // Initialize all common pins
    gpio->initializePins(commonPins);
    
    // Configure PWM for motor pins using ControlConfig values
    gpio->configurePWMPin(MOTOR_A_PIN, config.motor.motorFrequency, 8); // From config, 8-bit resolution
    gpio->configurePWMPin(MOTOR_B_PIN, config.motor.motorFrequency, 8); // From config, 8-bit resolution 
    
    // Configure PWM for audio pin using ControlConfig values
    gpio->configurePWMPin(AUDIO_PIN, config.feedback.audioFrequency, 8); // From config, 8-bit resolution
    
    // Configure I2C for screen (SDA=22, SCL=21)
    gpio->configureI2C(LCD_DATA_PIN, LCD_CLK_PIN);
    
    Serial.println("Common hardware pins configured using ControlConfig:");
    Serial.printf("  Sensors: L:%d C:%d R:%d (Thresholds: Black<%d, White>%d)\n", 
                  PHOTO_SENSOR_A_PIN, PHOTO_SENSOR_B_PIN, PHOTO_SENSOR_C_PIN,
                  config.sensors.blackThreshold, config.sensors.whiteThreshold);
    Serial.printf("  Button: %d (Control enabled: %s)\n", USER_BUTTON_PIN, 
                  config.system.enableButtonControl ? "Yes" : "No");
    Serial.printf("  Motors: A:%d B:%d (Frequency: %dHz, Base Speed: %d%%)\n", 
                  MOTOR_A_PIN, MOTOR_B_PIN, config.motor.motorFrequency, config.motor.baseSpeed);
    Serial.printf("  Audio: %d (Frequency: %dHz, Volume: %d%%, Enabled: %s)\n", 
                  AUDIO_PIN, config.feedback.audioFrequency, config.feedback.audioVolume,
                  config.feedback.enableAudio ? "Yes" : "No");
    Serial.printf("  Screen: SDA:%d SCL:%d (Display enabled: %s)\n", 
                  LCD_DATA_PIN, LCD_CLK_PIN, config.feedback.enableDisplay ? "Yes" : "No");
    
}

// TEST 1: CONTROL + UI INTEGRATION

void setupControlUITest() {
    Serial.println("=== CONTROL + UI INTEGRATION TEST ===");
    Serial.println("Testing: Button toggles system on/off with debouncing");
    Serial.println("Expected: Button press toggles system state reliably");
    Serial.println();
    
    initializeCommonComponents();
    
    // Initialize UserInterface
    ui = new UserInterface();
    ui->initialize();
    
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
    Serial.println("Testing: FSM state transitions with audio feedback");
    Serial.println("Expected: Distinct melodies for each state transition");
    Serial.println();
    
    initializeCommonComponents();
    
    // Initialize control system with speaker
    controlSystem = new ControlSubsystem();
    controlSystem->initialize();
    
    testStartTime = millis();
    testPhase = 0;
    
    Serial.println("Starting FSM audio test cycle...");
    Serial.println("States: Idle -> Forward -> TurnLeft -> TurnRight -> Stop");
}

void runControlSpeakerTest() {
    // Change state every 4 seconds
    if (millis() - lastTestUpdate > 4000) {
        
        const char* stateNames[] = {"IDLE", "FORWARD", "TURN_LEFT", "TURN_RIGHT", "STOP"};
        EventType events[] = {
            EventType::START, 
            EventType::FORWARD, 
            EventType::TURN_LEFT, 
            EventType::TURN_RIGHT, 
            EventType::STOP
        };
        
        if (testPhase < 5) {
            Serial.printf("Phase %d: Triggering %s state\n", testPhase + 1, stateNames[testPhase]);
            Serial.println("  -> Listen for audio melody...");
            
            // Create event and update control system
            Event testEvent(events[testPhase]);
            controlSystem->update();
            
            testPhase++;
        } else {
            // Reset cycle
            testPhase = 0;
            Serial.println("\n--- Audio test cycle complete, restarting ---\n");
        }
        
        lastTestUpdate = millis();
    }
    
    // Keep system running for audio processing
    controlSystem->update();
}

// TEST 3: CONTROL + SCREEN INTEGRATION

void setupControlScreenTest() {
    Serial.println("=== CONTROL + SCREEN INTEGRATION TEST ===");
    Serial.println("Testing: FSM state transitions with visual feedback");
    Serial.println("Expected: Screen displays correct state name immediately");
    Serial.println();
    
    initializeCommonComponents();
    
    // Initialize control system with screen
    controlSystem = new ControlSubsystem();
    controlSystem->initialize();
    
    testStartTime = millis();
    testPhase = 0;
    
    Serial.println("Starting FSM screen test cycle...");
    Serial.println("Watch screen for: STOP, FORWARD, LEFT, RIGHT messages");
}

void runControlScreenTest() {
    // Change state every 3 seconds for better visibility
    if (millis() - lastTestUpdate > 3000) {
        
        const char* stateNames[] = {"STOP", "FORWARD", "LEFT", "RIGHT"};
        EventType events[] = {
            EventType::STOP,
            EventType::FORWARD, 
            EventType::TURN_LEFT, 
            EventType::TURN_RIGHT
        };
        
        if (testPhase < 4) {
            Serial.printf("Phase %d: Display %s on screen\n", testPhase + 1, stateNames[testPhase]);
            Serial.println("  -> Check screen display now...");
            
            // Create event and update control system
            Event testEvent(events[testPhase]);
            controlSystem->update();
            
            testPhase++;
        } else {
            // Reset cycle
            testPhase = 0;
            Serial.println("\n--- Screen test cycle complete, restarting ---\n");
        }
        
        lastTestUpdate = millis();
    }
    
    // Keep system running for screen updates
    controlSystem->update();
}

// TEST 4: CONTROL + MOTORS INTEGRATION

void setupControlMotorsTest() {
    Serial.println("=== CONTROL + MOTORS INTEGRATION TEST ===");
    Serial.println("Testing: Motor control with PWM and differential steering");
    Serial.println("Expected: Motors respond to commands, differential turning works");
    Serial.println();
    
    initializeCommonComponents();
    
    // Initialize control system with motors
    controlSystem = new ControlSubsystem();
    controlSystem->initialize();
    
    testStartTime = millis();
    testPhase = 0;
    
    Serial.println("Starting motor control test sequence...");
    Serial.println("Sequence: Forward -> Turn Left -> Turn Right -> Stop");
}

void runControlMotorsTest() {
    // Change motor command every 3 seconds
    if (millis() - lastTestUpdate > 3000) {
        
        const char* commandNames[] = {"FORWARD", "TURN_LEFT", "TURN_RIGHT", "STOP"};
        EventType events[] = {
            EventType::FORWARD,
            EventType::TURN_LEFT, 
            EventType::TURN_RIGHT,
            EventType::STOP
        };
        
        if (testPhase < 4) {
            Serial.printf("Phase %d: Motor command %s\n", testPhase + 1, commandNames[testPhase]);
            
            switch (testPhase) {
                case 0:
                    Serial.println("  -> Both motors same speed (forward)");
                    break;
                case 1:
                    Serial.println("  -> Left motor slower (turn left)");
                    break;
                case 2:
                    Serial.println("  -> Right motor slower (turn right)");
                    break;
                case 3:
                    Serial.println("  -> Both motors stop");
                    break;
            }
            
            // Create event and update control system
            Event testEvent(events[testPhase]);
            controlSystem->update();
            
            testPhase++;
        } else {
            // Reset cycle
            testPhase = 0;
            Serial.println("\n--- Motor test cycle complete, restarting ---\n");
        }
        
        lastTestUpdate = millis();
    }
    
    // Keep system running for motor control
    controlSystem->update();
}

// TEST 5: CONTROL + SENSORS INTEGRATION

void setupControlSensorsTest() {
    Serial.println("=== CONTROL + SENSORS INTEGRATION TEST ===");
    Serial.println("Testing: LineDetector with PhotoSensor integration");
    Serial.println("Expected: Sensor readings stable, LineState changes with line position");
    Serial.println();
    
    initializeCommonComponents();
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Initialize sensors using ControlConfig thresholds
    int sensorThreshold = (config.sensors.blackThreshold + config.sensors.whiteThreshold) / 2; // Use midpoint
    sensorLeft = new PhotoSensor(PHOTO_SENSOR_A_PIN, sensorThreshold);
    sensorCenter = new PhotoSensor(PHOTO_SENSOR_B_PIN, sensorThreshold);
    sensorRight = new PhotoSensor(PHOTO_SENSOR_C_PIN, sensorThreshold);
    
    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();
    
    // Initialize LineDetector using ControlConfig thresholds
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight, 
                                   config.sensors.blackThreshold, config.sensors.whiteThreshold);
    
    testStartTime = millis();
    
    Serial.println("Sensor pins configured:");
    Serial.printf("  Left: Pin %d\n", PHOTO_SENSOR_A_PIN);
    Serial.printf("  Center: Pin %d\n", PHOTO_SENSOR_B_PIN);
    Serial.printf("  Right: Pin %d\n", PHOTO_SENSOR_C_PIN);
    Serial.println();
    Serial.println("Place line under sensors to test detection...");
}

void runControlSensorsTest() {
    // Read sensors every 300ms
    if (millis() - lastTestUpdate > 300) {
        
        // Get raw sensor readings
        int leftRaw = sensorLeft->readRaw();
        int centerRaw = sensorCenter->readRaw();
        int rightRaw = sensorRight->readRaw();
        
        // Get LineDetector analysis
        LineState lineState = lineDetector->detectLineState();
        float linePosition = lineDetector->calculateLinePosition();
        
        // Display readings
        Serial.printf("Sensors [L:%4d C:%4d R:%4d] | ", leftRaw, centerRaw, rightRaw);
        
        // Display line state
        Serial.print("State: ");
        switch (lineState) {
            case LineState::ON_LINE:
                Serial.print("ON_LINE");
                break;
            case LineState::TURN_LEFT:
                Serial.print("TURN_LEFT");
                break;
            case LineState::TURN_RIGHT:
                Serial.print("TURN_RIGHT");
                break;
            case LineState::OFF_LINE:
                Serial.print("OFF_LINE");
                break;
            case LineState::UNKNOWN:
                Serial.print("UNKNOWN");
                break;
        }
        
        Serial.printf(" | Position: %.3f\n", linePosition);
        
        lastTestUpdate = millis();
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
    Serial.println("Expected: Button controls AV system, FSM states match audio + screen");
    Serial.println();
    
    initializeCommonComponents();
    
    // Initialize UserInterface
    ui = new UserInterface();
    ui->initialize();
    
    // Initialize control system with AV components
    controlSystem = new ControlSubsystem();
    controlSystem->initialize();
    
    testStartTime = millis();
    testPhase = 0;
    systemOn = false;
    
    Serial.println("Press button to toggle AV system on/off...");
    Serial.println("When ON: FSM cycles every 5 seconds with coordinated audio + screen");
    Serial.println("When OFF: Audio stops, screen shows 'SYSTEM OFF'");
    Serial.println("System currently: OFF");
}

void runUIAVTest() {
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
                Serial.println("  -> AV system active - FSM cycling with audio + visual feedback");
                testPhase = 0; // Reset FSM cycle
            } else {
                Serial.println("  -> AV system inactive - audio stopped, screen off");
                
                // Stop audio and clear screen immediately
                speakerStop();              // Stop any playing audio
                showDirection(0);           // Display "STOP" to indicate system is off
                
                // Also send STOP event to control system for proper state management
                Event stopEvent(EventType::STOP);
                controlSystem->update();
            }
        }
        
        lastTestUpdate = millis();
    }
    
    // FSM cycling when system is ON
    static unsigned long lastFSMUpdate = 0;
    if (systemOn && millis() - lastFSMUpdate > 5000) { // 5-second cycles
        
        const char* stateNames[] = {"IDLE", "FORWARD", "TURN_LEFT", "TURN_RIGHT"};
        EventType events[] = {
            EventType::START,
            EventType::FORWARD, 
            EventType::TURN_LEFT, 
            EventType::TURN_RIGHT
        };
        
        // Cycle through states
        int currentState = testPhase % 4;
        Serial.printf("System ON - State: %s [Audio + Visual]\n", stateNames[currentState]);
        
        // Create event and update control system for AV feedback
        Event testEvent(events[currentState]);
        controlSystem->update();
        
        testPhase++;
        lastFSMUpdate = millis();
    }
    
    // Keep system running for AV processing
    if (systemOn) {
        controlSystem->update();
    }
}

// TEST 8: CONTROL + UI + AV + SENSORS

void setupUIAVSensorsTest() {
    Serial.println("=== PHASE II: CONTROL + UI + AV + SENSORS TEST ===");
    Serial.println("Testing: Sensor-driven AV feedback with button control");
    Serial.println("Expected: Sensors drive FSM states, AV matches line position");
    Serial.println();
    
    initializeCommonComponents();
    
    // Initialize UserInterface
    ui = new UserInterface();
    ui->initialize();
    
    // Initialize control system
    controlSystem = new ControlSubsystem();
    controlSystem->initialize();
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Initialize sensors using ControlConfig thresholds
    int sensorThreshold = (config.sensors.blackThreshold + config.sensors.whiteThreshold) / 2;
    sensorLeft = new PhotoSensor(PHOTO_SENSOR_A_PIN, sensorThreshold);
    sensorCenter = new PhotoSensor(PHOTO_SENSOR_B_PIN, sensorThreshold);
    sensorRight = new PhotoSensor(PHOTO_SENSOR_C_PIN, sensorThreshold);
    
    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();
    
    // Initialize LineDetector
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight, 
                                   config.sensors.blackThreshold, config.sensors.whiteThreshold);
    
    testStartTime = millis();
    systemOn = false;
    
    Serial.println("Press button to toggle sensor-driven AV system...");
    Serial.println("When ON: Sensors drive AV feedback in real-time");
    Serial.println("When OFF: Sensors monitor but no AV feedback");
    Serial.println("Place line under sensors to test...");
    Serial.println("System currently: OFF");
}

void runUIAVSensorsTest() {
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
                Serial.println("  -> Sensor-driven AV active - line following with feedback");
            } else {
                Serial.println("  -> AV disabled - sensors monitoring only");
            }
        }
        
        lastTestUpdate = millis();
    }
    
    // Sensor reading and AV feedback every 200ms
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate > 200) {
        
        // Get raw sensor readings
        int leftRaw = sensorLeft->readRaw();
        int centerRaw = sensorCenter->readRaw();
        int rightRaw = sensorRight->readRaw();
        
        // Get LineDetector analysis
        LineState lineState = lineDetector->detectLineState();
        float linePosition = lineDetector->calculateLinePosition();
        
        // Display sensor readings
        Serial.printf("Sensors [L:%4d C:%4d R:%4d] | ", leftRaw, centerRaw, rightRaw);
        
        // Display line state
        const char* stateStr = "UNKNOWN";
        const char* avStr = "None";
        
        switch (lineState) {
            case LineState::ON_LINE:
                stateStr = "ON_LINE";
                avStr = "Forward";
                break;
            case LineState::TURN_LEFT:
                stateStr = "TURN_LEFT";
                avStr = "Left";
                break;
            case LineState::TURN_RIGHT:
                stateStr = "TURN_RIGHT";
                avStr = "Right";
                break;
            case LineState::OFF_LINE:
                stateStr = "OFF_LINE";
                avStr = "Stop";
                break;
            case LineState::UNKNOWN:
                stateStr = "UNKNOWN";
                avStr = "None";
                break;
        }
        
        if (systemOn) {
            Serial.printf("System ON - State: %s [AV: %s] | Position: %.3f\n", stateStr, avStr, linePosition);
            
            // Update control system for AV feedback based on sensors
            controlSystem->update();
        } else {
            Serial.printf("System OFF - State: %s (monitoring only)\n", stateStr);
        }
        
        lastSensorUpdate = millis();
    }
}

// TEST 9: CONTROL + UI + AV + SENSORS + MOTORS

void setupUIAVSensorsMotorsTest() {
    Serial.println("=== PHASE II: CONTROL + UI + AV + SENSORS + MOTORS TEST ===");
    Serial.println("Testing: Complete line-following with AV feedback and motor control");
    Serial.println("Expected: Full autonomous operation with coordinated feedback");
    Serial.println();
    
    initializeCommonComponents();
    
    // Initialize UserInterface
    ui = new UserInterface();
    ui->initialize();
    
    // Initialize control system (includes motors)
    controlSystem = new ControlSubsystem();
    controlSystem->initialize();
    
    // Get configuration instance
    ControlConfig& config = ControlConfig::getInstance();
    
    // Initialize sensors using ControlConfig thresholds
    int sensorThreshold = (config.sensors.blackThreshold + config.sensors.whiteThreshold) / 2;
    sensorLeft = new PhotoSensor(PHOTO_SENSOR_A_PIN, sensorThreshold);
    sensorCenter = new PhotoSensor(PHOTO_SENSOR_B_PIN, sensorThreshold);
    sensorRight = new PhotoSensor(PHOTO_SENSOR_C_PIN, sensorThreshold);
    
    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();
    
    // Initialize LineDetector
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight, 
                                   config.sensors.blackThreshold, config.sensors.whiteThreshold);
    
    testStartTime = millis();
    systemOn = false;
    
    Serial.println("FULL LINE-FOLLOWING SYSTEM READY");
    Serial.println("Press button to toggle complete autonomous operation...");
    Serial.println("When ON: Full line-following with motors + AV feedback");
    Serial.println("When OFF: Motors stop immediately, sensors + AV monitor only");
    Serial.println("System currently: OFF - SAFE");
}

void runUIAVSensorsMotorsTest() {
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
                Serial.println("  -> Complete line-following engaged - motors + AV active");
                Serial.println("  -> WARNING: Robot will move autonomously!");
            } else {
                Serial.println("  -> Emergency stop - motors disabled, monitoring continues");
            }
        }
        
        lastTestUpdate = millis();
    }
    
    // Full system operation every 100ms for responsive control
    static unsigned long lastSystemUpdate = 0;
    if (millis() - lastSystemUpdate > 100) {
        
        // Get raw sensor readings
        int leftRaw = sensorLeft->readRaw();
        int centerRaw = sensorCenter->readRaw();
        int rightRaw = sensorRight->readRaw();
        
        // Get LineDetector analysis
        LineState lineState = lineDetector->detectLineState();
        float linePosition = lineDetector->calculateLinePosition();
        
        // Display sensor readings
        Serial.printf("Sensors [L:%4d C:%4d R:%4d] | ", leftRaw, centerRaw, rightRaw);
        
        // Display line state and system response
        const char* stateStr = "UNKNOWN";
        const char* motorStr = "Stop";
        const char* avStr = "None";
        
        switch (lineState) {
            case LineState::ON_LINE:
                stateStr = "ON_LINE";
                motorStr = "L:150 R:150";
                avStr = "Forward";
                break;
            case LineState::TURN_LEFT:
                stateStr = "TURN_LEFT";
                motorStr = "L:100 R:150";
                avStr = "Left";
                break;
            case LineState::TURN_RIGHT:
                stateStr = "TURN_RIGHT";
                motorStr = "L:150 R:100";
                avStr = "Right";
                break;
            case LineState::OFF_LINE:
                stateStr = "OFF_LINE";
                motorStr = "L:0 R:0";
                avStr = "Stop";
                break;
            case LineState::UNKNOWN:
                stateStr = "UNKNOWN";
                motorStr = "L:0 R:0";
                avStr = "None";
                break;
        }
        
        if (systemOn) {
            Serial.printf("System ON - LineState: %s -> Motors: %s [AV: %s]\n", stateStr, motorStr, avStr);
            
            // Update control system for full operation (sensors -> FSM -> motors + AV)
            controlSystem->update();
        } else {
            Serial.printf("System OFF - LineState: %s (motors disabled, monitoring only)\n", stateStr);
        }
        
        lastSystemUpdate = millis();
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
