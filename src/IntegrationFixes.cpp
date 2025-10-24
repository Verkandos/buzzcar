/*
 * BuzzCar Integration Test Fixes - Motor/Speaker  Investigation
 * 
 * These tests investigate the  coupling between motors and speakers
 * by varying initialization order and configurations.
 * 
 * Test Variations (based on Test 9 from hardware_demo_main.cpp):
 * TEST_FIX_1: Motors BEFORE speaker, continuous audio
 * TEST_FIX_2: Audio plays ONCE per state transition
 * TEST_FIX_3: Alternate pin mapping (Motor A=10, Motor B=0, Speaker=18), , continuous audio
 * TEST_FIX_4: Speaker initialized but NEVER plays
 * 
 * Enable ONLY ONE test at a time.
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

// TEST SELECTION - Enable ONLY ONE
const bool TEST_FIX_1 = true;   // Motors BEFORE speaker
const bool TEST_FIX_2 = false;  // Audio plays once per state transition (not continuous)
const bool TEST_FIX_3 = false;  // Different physical pins
const bool TEST_FIX_4 = false;  // Speaker initialized but does not play.

// TEST 2 SUB-OPTION - Enable ONLY ONE when TEST_FIX_2 = true
const bool TEST_2_MOTORS_FIRST = true;   // Initialize motors before speaker
const bool TEST_2_SPEAKER_FIRST = false; // Initialize speaker before motors

// TEST 3 SUB-OPTION - Enable ONLY ONE when TEST_FIX_3 = true
const bool TEST_3_MOTORS_FIRST = true;   // Initialize motors before speaker
const bool TEST_3_SPEAKER_FIRST = false; // Initialize speaker before motors

// TEST 4 SUB-OPTION - Enable ONLY ONE when TEST_FIX_4 = true
const bool TEST_4_MOTORS_FIRST = true;   // Initialize motors before speaker
const bool TEST_4_SENSORS_FIRST = false; // Initialize speaker before motors

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
bool systemOn = false;

// Utility Functions
bool validateTestSelection() {
    int activeTests = 0;
    if (TEST_FIX_1) activeTests++;
    if (TEST_FIX_2) activeTests++;
    if (TEST_FIX_3) activeTests++;
    if (TEST_FIX_4) activeTests++;
    
    if (activeTests != 1) {
        Serial.println("ERROR: Enable exactly ONE test. Current active tests: " + String(activeTests));
        return false;
    }
    return true;
}

// ========================================================================
// TEST FIX 1: Motors BEFORE Speaker
// ========================================================================

void setupTestFix1() {
    Serial.println("=== TEST FIX 1: MOTORS BEFORE SPEAKER ===");
    Serial.println("Hypothesis: Initializing motors first establishes stable Timer 0");
    Serial.println("Expected: Motors on Timer 0, then Speaker on Timer 1 (isolated)");
    Serial.println();
    
    gpio = &GPIOManager::getInstance();
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure all pins
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
    
    // STEP 1: Configure and initialize MOTORS FIRST
    Serial.println("STEP 1: Initializing motors first...");
    gpio->configurePWMPin(config.pins.motorA, config.motor.motorFrequency, 8);
    Serial.println("  -> Motor A PWM configured");
    gpio->configurePWMPin(config.pins.motorB, config.motor.motorFrequency, 8);
    Serial.println("  -> Motor B PWM configured");
    
    motorA = new Motor(config.pins.motorA);
    motorB = new Motor(config.pins.motorB);
    motorA->initialize();
    motorB->initialize();
    motorA->setMinimumStartPWM(0);
    motorB->setMinimumStartPWM(0);
    Serial.println("  -> Motors initialized and ready");
    
    // STEP 2: Configure and initialize SPEAKER SECOND
    Serial.println("STEP 2: Initializing speaker after motors...");
    gpio->configurePWMPin(config.pins.audio, config.feedback.audioFrequency, 10);
    Serial.println("  -> Speaker PWM configured");
    speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume);
    Serial.println("  -> Speaker initialized");
    
    // STEP 3: Initialize screen and sensors
    Serial.println("STEP 3: Initializing screen and sensors...");
    gpio->configureI2C(config.pins.lcdData, config.pins.lcdClk);
    screenBegin(config.pins.lcdData, config.pins.lcdClk);
    
    int sensorThreshold = (config.sensors.blackThreshold + config.sensors.whiteThreshold) / 2;
    sensorLeft = new PhotoSensor(config.pins.photoSensorA, sensorThreshold);
    sensorCenter = new PhotoSensor(config.pins.photoSensorB, sensorThreshold);
    sensorRight = new PhotoSensor(config.pins.photoSensorC, sensorThreshold);
    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();
    
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight,
                                   config.sensors.blackThreshold, config.sensors.whiteThreshold);
    
    // STEP 4: Initialize UI
    ui = new UserInterface();
    delay(10);
    ui->isButtonPressed();
    
    systemOn = false;
    showDirection(0);
    
    Serial.println("\n=== INITIALIZATION ORDER SUMMARY ===");
    Serial.println("1. Motors (Timer 0 @ 10kHz)");
    Serial.println("2. Speaker (Timer 1 @ variable)");
    Serial.println("3. Screen, Sensors, UI");
    Serial.println("=====================================\n");
    Serial.println("Press button to start test. Watch for  coupling...");
}

void runTestFix1() {
    static unsigned long lastFullUpdate = 0;
    
    // Button check
    if (millis() - lastTestUpdate > 50) {
        if (ui->wasButtonPressed()) {
            systemOn = !systemOn;
            ui->toggleSystem();
            
            Serial.printf("\n=== Button Pressed: System %s ===\n", systemOn ? "ON" : "OFF");
            
            if (!systemOn) {
                motorA->stop();
                motorB->stop();
                speakerStop();
                showDirection(0);
            }
        }
        lastTestUpdate = millis();
    }
    
    // Full system operation every 5 seconds
    if (systemOn && millis() - lastFullUpdate > 5000) {
        int leftRaw = sensorLeft->readRaw();
        int centerRaw = sensorCenter->readRaw();
        int rightRaw = sensorRight->readRaw();
        
        LineState lineState = lineDetector->detectLineState();
        float linePosition = lineDetector->calculateLinePosition();
        
        ControlConfig& config = ControlConfig::getInstance();
        
        const char* leftDetection = (leftRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                   (leftRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* centerDetection = (centerRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                     (centerRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* rightDetection = (rightRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                    (rightRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        
        int leftMotorSpeed, rightMotorSpeed, direction;
        const char* stateStr, *avStr;
        
        switch (lineState) {
            case LineState::ON_LINE:
                stateStr = "ON_LINE";
                avStr = "Forward";
                direction = 1;
                leftMotorSpeed = 50;
                rightMotorSpeed = 50;
                break;
            case LineState::TURN_LEFT:
                stateStr = "TURN_LEFT";
                avStr = "Left Turn";
                direction = 2;
                leftMotorSpeed = 0;
                rightMotorSpeed = 50;
                break;
            case LineState::TURN_RIGHT:
                stateStr = "TURN_RIGHT";
                avStr = "Right Turn";
                direction = 3;
                leftMotorSpeed = 50;
                rightMotorSpeed = 0;
                break;
            case LineState::OFF_LINE:
                stateStr = "OFF_LINE";
                avStr = "Stop (Off Line)";
                direction = 0;
                leftMotorSpeed = 0;
                rightMotorSpeed = 0;
                break;
            case LineState::UNKNOWN:
                stateStr = "UNKNOWN";
                avStr = "Stop (Unknown)";
                direction = 0;
                leftMotorSpeed = 0;
                rightMotorSpeed = 0;
                break;
        }
        
        Serial.println("\n=== SYSTEM UPDATE (Motors Init First) ===");
        Serial.printf("Sensors: L:%d C:%d R:%d | Detection: L:%s C:%s R:%s\n",
                     leftRaw, centerRaw, rightRaw, leftDetection, centerDetection, rightDetection);
        Serial.printf("Line State: %s | Position: %.3f\n", stateStr, linePosition);
        Serial.printf("Motor Command: L=%d%%, R=%d%%\n", leftMotorSpeed, rightMotorSpeed);
        
        motorA->setSpeed(leftMotorSpeed);
        motorB->setSpeed(rightMotorSpeed);
        
        Serial.printf("Motor A: Speed=%d%%, PWM=%d | Motor B: Speed=%d%%, PWM=%d\n",
                     motorA->getCurrentSpeed(), motorA->getCurrentPWM(),
                     motorB->getCurrentSpeed(), motorB->getCurrentPWM());
        
        startMelodyForDirection(direction);
        showDirection(direction);
        
        Serial.println("=========================================\n");
        
        lastFullUpdate = millis();
    }
    
    if (systemOn) {
        serviceMelody();
    }
}

// ========================================================================
// TEST FIX 2: Audio plays ONCE per state transition (not continuous)
// ========================================================================

void setupTestFix2() {
    Serial.println("=== TEST FIX 2: AUDIO ONCE PER STATE TRANSITION ===");
    
    // Validate sub-option selection
    int activeModes = 0;
    if (TEST_2_MOTORS_FIRST) activeModes++;
    if (TEST_2_SPEAKER_FIRST) activeModes++;
    
    if (activeModes != 1) {
        Serial.println("ERROR: Enable exactly ONE initialization order for Test 2!");
        Serial.printf("Current active modes: %d\n", activeModes);
        while (true) { delay(1000); }
    }
    
    if (TEST_2_MOTORS_FIRST) {
        Serial.println("Sub-option: MOTORS initialized BEFORE speaker");
    } else {
        Serial.println("Sub-option: SPEAKER initialized BEFORE motors");
    }
    
    Serial.println("Behavior: Speaker plays once per state change (not continuous)");
    Serial.println("Expected: Clean state transitions without continuous audio loop");
    Serial.println();
    
    gpio = &GPIOManager::getInstance();
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure all pins
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
    
    if (TEST_2_MOTORS_FIRST) {
        // MOTORS FIRST
        Serial.println("STEP 1: Initializing motors first...");
        gpio->configurePWMPin(config.pins.motorA, config.motor.motorFrequency, 8);
        Serial.println("  -> Motor A PWM configured");
        gpio->configurePWMPin(config.pins.motorB, config.motor.motorFrequency, 8);
        Serial.println("  -> Motor B PWM configured");
        
        motorA = new Motor(config.pins.motorA);
        motorB = new Motor(config.pins.motorB);
        motorA->initialize();
        motorB->initialize();
        motorA->setMinimumStartPWM(0);
        motorB->setMinimumStartPWM(0);
        Serial.println("  -> Motors initialized and ready");
        
        Serial.println("STEP 2: Initializing speaker after motors...");
        gpio->configurePWMPin(config.pins.audio, config.feedback.audioFrequency, 10);
        Serial.println("  -> Speaker PWM configured");
        speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume);
        Serial.println("  -> Speaker initialized");
    } else {
        // SPEAKER FIRST
        Serial.println("STEP 1: Initializing speaker first...");
        gpio->configurePWMPin(config.pins.audio, config.feedback.audioFrequency, 10);
        Serial.println("  -> Speaker PWM configured");
        speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume);
        Serial.println("  -> Speaker initialized");
        
        Serial.println("STEP 2: Initializing motors after speaker...");
        gpio->configurePWMPin(config.pins.motorA, config.motor.motorFrequency, 8);
        Serial.println("  -> Motor A PWM configured");
        gpio->configurePWMPin(config.pins.motorB, config.motor.motorFrequency, 8);
        Serial.println("  -> Motor B PWM configured");
        
        motorA = new Motor(config.pins.motorA);
        motorB = new Motor(config.pins.motorB);
        motorA->initialize();
        motorB->initialize();
        motorA->setMinimumStartPWM(0);
        motorB->setMinimumStartPWM(0);
        Serial.println("  -> Motors initialized and ready");
    }
    
    // STEP 3: Initialize screen and sensors
    Serial.println("STEP 3: Initializing screen and sensors...");
    gpio->configureI2C(config.pins.lcdData, config.pins.lcdClk);
    screenBegin(config.pins.lcdData, config.pins.lcdClk);
    
    int sensorThreshold = (config.sensors.blackThreshold + config.sensors.whiteThreshold) / 2;
    sensorLeft = new PhotoSensor(config.pins.photoSensorA, sensorThreshold);
    sensorCenter = new PhotoSensor(config.pins.photoSensorB, sensorThreshold);
    sensorRight = new PhotoSensor(config.pins.photoSensorC, sensorThreshold);
    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();
    
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight,
                                   config.sensors.blackThreshold, config.sensors.whiteThreshold);
    
    // STEP 4: Initialize UI
    ui = new UserInterface();
    delay(10);
    ui->isButtonPressed();
    
    systemOn = false;
    showDirection(0);
    
    Serial.println("\n=== INITIALIZATION ORDER SUMMARY ===");
    if (TEST_2_MOTORS_FIRST) {
        Serial.println("1. Motors (Timer 0 @ 10kHz)");
        Serial.println("2. Speaker (Timer 1 @ variable)");
    } else {
        Serial.println("1. Speaker (Timer 1 @ variable)");
        Serial.println("2. Motors (Timer 0 @ 10kHz)");
    }
    Serial.println("3. Screen, Sensors, UI");
    Serial.println("=====================================\n");
    Serial.println("Press button to start test. Audio plays ONCE per state change...");
}

void runTestFix2() {
    static unsigned long lastFullUpdate = 0;
    static LineState previousLineState = LineState::UNKNOWN;
    static bool audioPlaying = false;
    
    // Button check
    if (millis() - lastTestUpdate > 50) {
        if (ui->wasButtonPressed()) {
            systemOn = !systemOn;
            ui->toggleSystem();
            
            Serial.printf("\n=== Button Pressed: System %s ===\n", systemOn ? "ON" : "OFF");
            
            if (!systemOn) {
                motorA->stop();
                motorB->stop();
                speakerStop();
                showDirection(0);
                previousLineState = LineState::UNKNOWN;
                audioPlaying = false;
            }
        }
        lastTestUpdate = millis();
    }
    
    // Full system operation every 5 seconds
    if (systemOn && millis() - lastFullUpdate > 5000) {
        int leftRaw = sensorLeft->readRaw();
        int centerRaw = sensorCenter->readRaw();
        int rightRaw = sensorRight->readRaw();
        
        LineState lineState = lineDetector->detectLineState();
        float linePosition = lineDetector->calculateLinePosition();
        
        ControlConfig& config = ControlConfig::getInstance();
        
        const char* leftDetection = (leftRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                   (leftRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* centerDetection = (centerRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                     (centerRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* rightDetection = (rightRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                    (rightRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        
        int leftMotorSpeed, rightMotorSpeed, direction;
        const char* stateStr, *avStr;
        
        switch (lineState) {
            case LineState::ON_LINE:
                stateStr = "ON_LINE";
                avStr = "Forward";
                direction = 1;
                leftMotorSpeed = 50;
                rightMotorSpeed = 50;
                break;
            case LineState::TURN_LEFT:
                stateStr = "TURN_LEFT";
                avStr = "Left Turn";
                direction = 2;
                leftMotorSpeed = 0;
                rightMotorSpeed = 50;
                break;
            case LineState::TURN_RIGHT:
                stateStr = "TURN_RIGHT";
                avStr = "Right Turn";
                direction = 3;
                leftMotorSpeed = 50;
                rightMotorSpeed = 0;
                break;
            case LineState::OFF_LINE:
                stateStr = "OFF_LINE";
                avStr = "Stop (Off Line)";
                direction = 0;
                leftMotorSpeed = 0;
                rightMotorSpeed = 0;
                break;
            case LineState::UNKNOWN:
                stateStr = "UNKNOWN";
                avStr = "Stop (Unknown)";
                direction = 0;
                leftMotorSpeed = 0;
                rightMotorSpeed = 0;
                break;
        }
        
        Serial.println("\n=== SYSTEM UPDATE (Audio Once Per State) ===");
        Serial.printf("Sensors: L:%d C:%d R:%d | Detection: L:%s C:%s R:%s\n",
                     leftRaw, centerRaw, rightRaw, leftDetection, centerDetection, rightDetection);
        Serial.printf("Line State: %s | Position: %.3f\n", stateStr, linePosition);
        Serial.printf("Motor Command: L=%d%%, R=%d%%\n", leftMotorSpeed, rightMotorSpeed);
        
        motorA->setSpeed(leftMotorSpeed);
        motorB->setSpeed(rightMotorSpeed);
        
        Serial.printf("Motor A: Speed=%d%%, PWM=%d | Motor B: Speed=%d%%, PWM=%d\n",
                     motorA->getCurrentSpeed(), motorA->getCurrentPWM(),
                     motorB->getCurrentSpeed(), motorB->getCurrentPWM());
        
        // Only play audio if state has changed
        if (lineState != previousLineState) {
            Serial.printf("STATE CHANGE DETECTED: %s -> %s\n", 
                         (previousLineState == LineState::ON_LINE ? "ON_LINE" :
                          previousLineState == LineState::TURN_LEFT ? "TURN_LEFT" :
                          previousLineState == LineState::TURN_RIGHT ? "TURN_RIGHT" :
                          previousLineState == LineState::OFF_LINE ? "OFF_LINE" : "UNKNOWN"),
                         stateStr);
            Serial.printf("Playing audio ONCE for: %s\n", avStr);
            startMelodyForDirection(direction);
            audioPlaying = true;
            previousLineState = lineState;
        } else {
            Serial.printf("No state change - audio NOT played (still %s)\n", stateStr);
            audioPlaying = false;
        }
        
        showDirection(direction);
        
        Serial.println("=============================================\n");
        
        lastFullUpdate = millis();
    }
    
    // Only service melody if audio is currently playing
    if (systemOn && audioPlaying) {
        serviceMelody();
    }
}

// ========================================================================
// TEST FIX 3: Alternate Pin Mapping (Motor A=10, Motor B=0, Speaker=18)
// ========================================================================

void setupTestFix3() {
    Serial.println("=== TEST FIX 3: ALTERNATE PIN MAPPING ===");
    
    // Validate sub-option selection
    int activeModes = 0;
    if (TEST_3_MOTORS_FIRST) activeModes++;
    if (TEST_3_SPEAKER_FIRST) activeModes++;
    
    if (activeModes != 1) {
        Serial.println("ERROR: Enable exactly ONE initialization order for Test 3!");
        Serial.printf("Current active modes: %d\n", activeModes);
        while (true) { delay(1000); }
    }
    
    if (TEST_3_MOTORS_FIRST) {
        Serial.println("Sub-option: MOTORS initialized BEFORE speaker");
    } else {
        Serial.println("Sub-option: SPEAKER initialized BEFORE motors");
    }
    
    Serial.println("Testing different GPIO pins");
    Serial.println("Pin Configuration: Motor A=10, Motor B=0, Speaker=18");
    Serial.println("Expected: Motors share Timer 0, Speaker uses Timer 1");
    Serial.println();
    
    gpio = &GPIOManager::getInstance();
    ControlConfig& config = ControlConfig::getInstance();
    
    // Define alternate pins
    const int ALT_MOTOR_A_PIN = 10;
    const int ALT_MOTOR_B_PIN = 0;
    const int ALT_AUDIO_PIN = 18;
    
    // Configure all pins (using alternate motor/audio pins)
    std::map<int, std::string> fullSystemPins = {
        {config.pins.userButton, "digital_input_pullup"},
        {ALT_AUDIO_PIN, "digital_output"},
        {config.pins.lcdData, "digital_output"},
        {config.pins.lcdClk, "digital_output"},
        {config.pins.photoSensorA, "analog_input"},
        {config.pins.photoSensorB, "analog_input"},
        {config.pins.photoSensorC, "analog_input"},
        {ALT_MOTOR_A_PIN, "digital_output"},
        {ALT_MOTOR_B_PIN, "digital_output"}
    };
    gpio->initializePins(fullSystemPins);
    
    if (TEST_3_MOTORS_FIRST) {
        // MOTORS FIRST
        Serial.println("STEP 1: Initializing motors first...");
        gpio->configurePWMPin(ALT_MOTOR_A_PIN, config.motor.motorFrequency, 8);
        Serial.printf("  -> Motor A PWM configured on pin %d\n", ALT_MOTOR_A_PIN);
        gpio->configurePWMPin(ALT_MOTOR_B_PIN, config.motor.motorFrequency, 8);
        Serial.printf("  -> Motor B PWM configured on pin %d\n", ALT_MOTOR_B_PIN);
        
        motorA = new Motor(ALT_MOTOR_A_PIN);
        motorB = new Motor(ALT_MOTOR_B_PIN);
        motorA->initialize();
        motorB->initialize();
        motorA->setMinimumStartPWM(0);
        motorB->setMinimumStartPWM(0);
        Serial.println("  -> Motors initialized and ready");
        
        Serial.println("STEP 2: Initializing speaker after motors...");
        gpio->configurePWMPin(ALT_AUDIO_PIN, config.feedback.audioFrequency, 10);
        Serial.printf("  -> Speaker PWM configured on pin %d\n", ALT_AUDIO_PIN);
        speakerBegin(ALT_AUDIO_PIN, 2, 10, config.feedback.audioVolume);
        Serial.println("  -> Speaker initialized");
    } else {
        // SPEAKER FIRST
        Serial.println("STEP 1: Initializing speaker first...");
        gpio->configurePWMPin(ALT_AUDIO_PIN, config.feedback.audioFrequency, 10);
        Serial.printf("  -> Speaker PWM configured on pin %d\n", ALT_AUDIO_PIN);
        speakerBegin(ALT_AUDIO_PIN, 2, 10, config.feedback.audioVolume);
        Serial.println("  -> Speaker initialized");
        
        Serial.println("STEP 2: Initializing motors after speaker...");
        gpio->configurePWMPin(ALT_MOTOR_A_PIN, config.motor.motorFrequency, 8);
        Serial.printf("  -> Motor A PWM configured on pin %d\n", ALT_MOTOR_A_PIN);
        gpio->configurePWMPin(ALT_MOTOR_B_PIN, config.motor.motorFrequency, 8);
        Serial.printf("  -> Motor B PWM configured on pin %d\n", ALT_MOTOR_B_PIN);
        
        motorA = new Motor(ALT_MOTOR_A_PIN);
        motorB = new Motor(ALT_MOTOR_B_PIN);
        motorA->initialize();
        motorB->initialize();
        motorA->setMinimumStartPWM(0);
        motorB->setMinimumStartPWM(0);
        Serial.println("  -> Motors initialized and ready");
    }
    
    // STEP 3: Initialize screen and sensors
    Serial.println("STEP 3: Initializing screen and sensors...");
    gpio->configureI2C(config.pins.lcdData, config.pins.lcdClk);
    screenBegin(config.pins.lcdData, config.pins.lcdClk);
    
    int sensorThreshold = (config.sensors.blackThreshold + config.sensors.whiteThreshold) / 2;
    sensorLeft = new PhotoSensor(config.pins.photoSensorA, sensorThreshold);
    sensorCenter = new PhotoSensor(config.pins.photoSensorB, sensorThreshold);
    sensorRight = new PhotoSensor(config.pins.photoSensorC, sensorThreshold);
    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();
    
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight,
                                   config.sensors.blackThreshold, config.sensors.whiteThreshold);
    
    // STEP 4: Initialize UI
    ui = new UserInterface();
    delay(10);
    ui->isButtonPressed();
    
    systemOn = false;
    showDirection(0);
    
    Serial.println("\n=== INITIALIZATION ORDER SUMMARY ===");

    Serial.println("3. Screen, Sensors, UI");
    Serial.println("\n=== PIN CONFIGURATION SUMMARY ===");
    Serial.printf("Motor A: Pin %d (was 20) -> Timer 0, Channel 1\n", ALT_MOTOR_A_PIN);
    Serial.printf("Motor B: Pin %d (was 19) -> Timer 0, Channel 0\n", ALT_MOTOR_B_PIN);
    Serial.printf("Speaker: Pin %d (was 23) -> Timer 1, Channel 2\n", ALT_AUDIO_PIN);
    Serial.printf("Sensors: Pins %d, %d, %d (unchanged)\n", 
                  config.pins.photoSensorA, config.pins.photoSensorB, config.pins.photoSensorC);
    Serial.printf("Screen: Pins %d, %d (unchanged)\n", config.pins.lcdData, config.pins.lcdClk);
    Serial.println("==================================");
    Serial.println("\nWatch GPIOManager output to confirm Timer/Channel assignments!");
    Serial.println("Tests different physical pins\n");
    Serial.println("Press button to start test...");
}

void runTestFix3() {
    static unsigned long lastFullUpdate = 0;
    
    // Button check
    if (millis() - lastTestUpdate > 50) {
        if (ui->wasButtonPressed()) {
            systemOn = !systemOn;
            ui->toggleSystem();
            
            Serial.printf("\n=== Button Pressed: System %s ===\n", systemOn ? "ON" : "OFF");
            
            if (!systemOn) {
                motorA->stop();
                motorB->stop();
                speakerStop();
                showDirection(0);
            }
        }
        lastTestUpdate = millis();
    }
    
    // Full system operation every 5 seconds
    if (systemOn && millis() - lastFullUpdate > 5000) {
        int leftRaw = sensorLeft->readRaw();
        int centerRaw = sensorCenter->readRaw();
        int rightRaw = sensorRight->readRaw();
        
        LineState lineState = lineDetector->detectLineState();
        float linePosition = lineDetector->calculateLinePosition();
        
        ControlConfig& config = ControlConfig::getInstance();
        
        const char* leftDetection = (leftRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                   (leftRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* centerDetection = (centerRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                     (centerRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* rightDetection = (rightRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                    (rightRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        
        int leftMotorSpeed, rightMotorSpeed, direction;
        const char* stateStr, *avStr;
        
        switch (lineState) {
            case LineState::ON_LINE:
                stateStr = "ON_LINE";
                avStr = "Forward";
                direction = 1;
                leftMotorSpeed = 50;
                rightMotorSpeed = 50;
                break;
            case LineState::TURN_LEFT:
                stateStr = "TURN_LEFT";
                avStr = "Left Turn";
                direction = 2;
                leftMotorSpeed = 0;
                rightMotorSpeed = 50;
                break;
            case LineState::TURN_RIGHT:
                stateStr = "TURN_RIGHT";
                avStr = "Right Turn";
                direction = 3;
                leftMotorSpeed = 50;
                rightMotorSpeed = 0;
                break;
            case LineState::OFF_LINE:
                stateStr = "OFF_LINE";
                avStr = "Stop (Off Line)";
                direction = 0;
                leftMotorSpeed = 0;
                rightMotorSpeed = 0;
                break;
            case LineState::UNKNOWN:
                stateStr = "UNKNOWN";
                avStr = "Stop (Unknown)";
                direction = 0;
                leftMotorSpeed = 0;
                rightMotorSpeed = 0;
                break;
        }
        
        Serial.println("\n=== SYSTEM UPDATE (Alternate Pins: 10, 0, 18) ===");
        Serial.printf("Sensors: L:%d C:%d R:%d | Detection: L:%s C:%s R:%s\n",
                     leftRaw, centerRaw, rightRaw, leftDetection, centerDetection, rightDetection);
        Serial.printf("Line State: %s | Position: %.3f\n", stateStr, linePosition);
        Serial.printf("Motor Command: L=%d%%, R=%d%%\n", leftMotorSpeed, rightMotorSpeed);
        
        motorA->setSpeed(leftMotorSpeed);
        motorB->setSpeed(rightMotorSpeed);
        
        Serial.printf("Motor A: Speed=%d%%, PWM=%d | Motor B: Speed=%d%%, PWM=%d\n",
                     motorA->getCurrentSpeed(), motorA->getCurrentPWM(),
                     motorB->getCurrentSpeed(), motorB->getCurrentPWM());
        
        startMelodyForDirection(direction);
        showDirection(direction);
        
        Serial.println("==================================================\n");
        
        lastFullUpdate = millis();
    }
    
    if (systemOn) {
        serviceMelody();
    }
}

// ========================================================================
// TEST FIX 4: Speaker initialized but NEVER plays audio
// ========================================================================

void setupTestFix4() {
    Serial.println("=== TEST FIX 4: SPEAKER INITIALIZED BUT NEVER PLAYS ===");
    
    // Validate sub-option selection
    int activeModes = 0;
    if (TEST_4_MOTORS_FIRST) activeModes++;
    if (TEST_4_SENSORS_FIRST) activeModes++;
    
    if (activeModes != 1) {
        Serial.println("ERROR: Enable exactly ONE initialization order for Test 4!");
        Serial.printf("Current active modes: %d\n", activeModes);
        while (true) { delay(1000); }
    }
    
    if (TEST_4_MOTORS_FIRST) {
        Serial.println("Sub-option: MOTORS initialized BEFORE speaker");
    } else {
        Serial.println("Sub-option: SPEAKER initialized BEFORE motors");
    }
    
    
    gpio = &GPIOManager::getInstance();
    ControlConfig& config = ControlConfig::getInstance();
    
    // Configure all pins
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
    
    if (TEST_4_MOTORS_FIRST) {
        // MOTORS FIRST
        Serial.println("STEP 1: Initializing motors first...");
        gpio->configurePWMPin(config.pins.motorA, config.motor.motorFrequency, 8);
        Serial.println("  -> Motor A PWM configured");
        gpio->configurePWMPin(config.pins.motorB, config.motor.motorFrequency, 8);
        Serial.println("  -> Motor B PWM configured");
        
        motorA = new Motor(config.pins.motorA);
        motorB = new Motor(config.pins.motorB);
        motorA->initialize();
        motorB->initialize();
        motorA->setMinimumStartPWM(0);
        motorB->setMinimumStartPWM(0);
        Serial.println("  -> Motors initialized and ready");
        
        Serial.println("STEP 2: Initializing speaker after motors...");
        gpio->configurePWMPin(config.pins.audio, config.feedback.audioFrequency, 10);
        Serial.println("  -> Speaker PWM configured");
        speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume);
        Serial.println("  -> Speaker initialized (BUT WILL NEVER PLAY)");
    } else {
        // SPEAKER FIRST
        Serial.println("STEP 1: Initializing speaker first...");
        gpio->configurePWMPin(config.pins.audio, config.feedback.audioFrequency, 10);
        Serial.println("  -> Speaker PWM configured");
        speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume);
        Serial.println("  -> Speaker initialized (BUT WILL NEVER PLAY)");
        
        Serial.println("STEP 2: Initializing motors after speaker...");
        gpio->configurePWMPin(config.pins.motorA, config.motor.motorFrequency, 8);
        Serial.println("  -> Motor A PWM configured");
        gpio->configurePWMPin(config.pins.motorB, config.motor.motorFrequency, 8);
        Serial.println("  -> Motor B PWM configured");
        
        motorA = new Motor(config.pins.motorA);
        motorB = new Motor(config.pins.motorB);
        motorA->initialize();
        motorB->initialize();
        motorA->setMinimumStartPWM(0);
        motorB->setMinimumStartPWM(0);
        Serial.println("  -> Motors initialized and ready");
    }
    
    // STEP 3: Initialize screen and sensors
    Serial.println("STEP 3: Initializing screen and sensors...");
    gpio->configureI2C(config.pins.lcdData, config.pins.lcdClk);
    screenBegin(config.pins.lcdData, config.pins.lcdClk);
    
    int sensorThreshold = (config.sensors.blackThreshold + config.sensors.whiteThreshold) / 2;
    sensorLeft = new PhotoSensor(config.pins.photoSensorA, sensorThreshold);
    sensorCenter = new PhotoSensor(config.pins.photoSensorB, sensorThreshold);
    sensorRight = new PhotoSensor(config.pins.photoSensorC, sensorThreshold);
    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();
    
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight,
                                   config.sensors.blackThreshold, config.sensors.whiteThreshold);
    
    // STEP 4: Initialize UI
    ui = new UserInterface();
    delay(10);
    ui->isButtonPressed();
    
    systemOn = false;
    showDirection(0);
    
    Serial.println("\n=== INITIALIZATION ORDER SUMMARY ===");
    if (TEST_4_MOTORS_FIRST) {
        Serial.println("1. Motors (Timer 0 @ 10kHz)");
        Serial.println("2. Speaker (Timer 1 @ variable) - INITIALIZED ONLY");
    } else {
        Serial.println("1. Speaker (Timer 1 @ variable) - INITIALIZED ONLY");
        Serial.println("2. Motors (Timer 0 @ 10kHz)");
    }
    Serial.println("3. Screen, Sensors, UI");
    Serial.println("=====================================");
    Serial.println("CRITICAL: startMelodyForDirection() will NOT be called");
    Serial.println("Speaker is configured and ready but silent.");
    Serial.println("=====================================\n");
    Serial.println("Press button to start test. without audio playback...");
}

void runTestFix4() {
    static unsigned long lastFullUpdate = 0;
    
    // Button check
    if (millis() - lastTestUpdate > 50) {
        if (ui->wasButtonPressed()) {
            systemOn = !systemOn;
            ui->toggleSystem();
            
            Serial.printf("\n=== Button Pressed: System %s ===\n", systemOn ? "ON" : "OFF");
            
            if (!systemOn) {
                motorA->stop();
                motorB->stop();
                // Note: Speaker never plays, so no need to stop it
                showDirection(0);
            }
        }
        lastTestUpdate = millis();
    }
    
    // Full system operation every 5 seconds
    if (systemOn && millis() - lastFullUpdate > 5000) {
        int leftRaw = sensorLeft->readRaw();
        int centerRaw = sensorCenter->readRaw();
        int rightRaw = sensorRight->readRaw();
        
        LineState lineState = lineDetector->detectLineState();
        float linePosition = lineDetector->calculateLinePosition();
        
        ControlConfig& config = ControlConfig::getInstance();
        
        const char* leftDetection = (leftRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                   (leftRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* centerDetection = (centerRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                     (centerRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        const char* rightDetection = (rightRaw > config.sensors.blackThreshold) ? "BLACK" : 
                                    (rightRaw < config.sensors.whiteThreshold) ? "WHITE" : "GRAY";
        
        int leftMotorSpeed, rightMotorSpeed, direction;
        const char* stateStr, *avStr;
        
        switch (lineState) {
            case LineState::ON_LINE:
                stateStr = "ON_LINE";
                avStr = "Forward (Silent)";
                direction = 1;
                leftMotorSpeed = 50;
                rightMotorSpeed = 50;
                break;
            case LineState::TURN_LEFT:
                stateStr = "TURN_LEFT";
                avStr = "Left Turn (Silent)";
                direction = 2;
                leftMotorSpeed = 0;
                rightMotorSpeed = 50;
                break;
            case LineState::TURN_RIGHT:
                stateStr = "TURN_RIGHT";
                avStr = "Right Turn (Silent)";
                direction = 3;
                leftMotorSpeed = 50;
                rightMotorSpeed = 0;
                break;
            case LineState::OFF_LINE:
                stateStr = "OFF_LINE";
                avStr = "Stop (Off Line, Silent)";
                direction = 0;
                leftMotorSpeed = 0;
                rightMotorSpeed = 0;
                break;
            case LineState::UNKNOWN:
                stateStr = "UNKNOWN";
                avStr = "Stop (Unknown, Silent)";
                direction = 0;
                leftMotorSpeed = 0;
                rightMotorSpeed = 0;
                break;
        }
        
        Serial.println("\n=== SYSTEM UPDATE (Speaker Silent) ===");
        Serial.printf("Sensors: L:%d C:%d R:%d | Detection: L:%s C:%s R:%s\n",
                     leftRaw, centerRaw, rightRaw, leftDetection, centerDetection, rightDetection);
        Serial.printf("Line State: %s | Position: %.3f\n", stateStr, linePosition);
        Serial.printf("Motor Command: L=%d%%, R=%d%%\n", leftMotorSpeed, rightMotorSpeed);
        
        motorA->setSpeed(leftMotorSpeed);
        motorB->setSpeed(rightMotorSpeed);
        
        Serial.printf("Motor A: Speed=%d%%, PWM=%d | Motor B: Speed=%d%%, PWM=%d\n",
                     motorA->getCurrentSpeed(), motorA->getCurrentPWM(),
                     motorB->getCurrentSpeed(), motorB->getCurrentPWM());
        
        // CRITICAL: NO AUDIO PLAYBACK
        // startMelodyForDirection(direction); // INTENTIONALLY COMMENTED OUT
        Serial.printf("Audio Action: NONE (would be '%s' but speaker is silent)\n", avStr);
        
        showDirection(direction);
        
        Serial.println("=======================================\n");
        
        lastFullUpdate = millis();
    }
    
    // CRITICAL: serviceMelody() is NEVER called
    // This ensures no audio is ever played during this test
}

// ========================================================================
// MAIN SETUP AND LOOP
// ========================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    delay(1000);
    
    Serial.println("BuzzCar Integration Test Fixes - Motor/Speaker  Investigation");
    Serial.println("==================================================================");
    
    if (!validateTestSelection()) {
        Serial.println("HALTING - Fix test selection");
        while (true) { delay(1000); }
    }
    
    setCpuFrequencyMhz(160);
    Serial.printf("CPU Frequency: %d MHz\n\n", getCpuFrequencyMhz());
    
    if (TEST_FIX_1) {
        setupTestFix1();
    } else if (TEST_FIX_2) {
        setupTestFix2();
    } else if (TEST_FIX_3) {
        setupTestFix3();
    } else if (TEST_FIX_4) {
        setupTestFix4();
    }
    
    Serial.println("\nTest initialized. Starting execution...\n");
}

void loop() {
    if (TEST_FIX_1) {
        runTestFix1();
    } else if (TEST_FIX_2) {
        runTestFix2();
    } else if (TEST_FIX_3) {
        runTestFix3();
    } else if (TEST_FIX_4) {
        runTestFix4();
    }
    
    delay(10);
}
