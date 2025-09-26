/*
 * BuzzCar Hardware Integration Test
 * 
 * Pairwise integration testing for hardware components:
 * - Motors (10kHz PWM, 10%-75% duty cycles)
 * - PhotoSensors (analog readings)
 * - User Button (digital input with debouncing)
 * - Power (Confirmation of power system via onboard LED)
 * - Audio (TBD) 
 * - Visual (TBD)
 */

#include <Arduino.h>
#include "GPIOManager.hpp"
#include "LineDetector.hpp"
#include "PhotoSensor.hpp"

PhotoSensor* testSensorLeft = nullptr;
PhotoSensor* testSensorCenter = nullptr;
PhotoSensor* testSensorRight = nullptr;
LineDetector* testLineDetector = nullptr;

// TEST CONTROL SWITCHES - MANUALLY SET THESE (ONLY ONE TRUE AT A TIME)
bool MOTOR_TEST = false;        // Motor PWM duty cycle progression test
bool PHOTOSENSOR_TEST = false; // PhotoSensor analog reading test
bool BUTTON_TEST = false;      // User button input test
bool POWER_TEST = false;       // Power system test
bool AUDIO_TEST = true;       // Audio output test (TBD)
bool VISUAL_TEST = true;      // Visual output test (TBD)

// TEST PARAMETERS

// Motor Test Parameters
const int MOTOR_FREQUENCY = 10000;  // 10kHz
const int TEST_DURATION_MS = 5000;   // 5 seconds per test
const int DUTY_STEPS[] = {10, 20, 30, 40, 50, 60, 70, 75, 0}; // Duty cycle percentages
const int NUM_DUTY_STEPS = sizeof(DUTY_STEPS) / sizeof(DUTY_STEPS[0]);

// PhotoSensor Test Parameters
const int SENSOR_READ_INTERVAL_MS = 500; // Read sensors every 500ms
const int SENSOR_SAMPLES = 10;           // Number of samples to average

// Button Test Parameters
const int BUTTON_DEBOUNCE_MS = 50;       // Debounce delay
const int BUTTON_READ_INTERVAL_MS = 100; // Check button every 100ms

// Global variables
GPIOManager* gpio;
unsigned long lastTestTime = 0;
int currentTestStep = 0;
bool lastButtonState = false;
unsigned long lastButtonTime = 0;

// UTILITY FUNCTIONS
bool validateTestSwitches() {
    int activeTests = 0;
    if (MOTOR_TEST) activeTests++;
    if (PHOTOSENSOR_TEST) activeTests++;
    if (BUTTON_TEST) activeTests++;
    if (POWER_TEST) activeTests++;
    if (AUDIO_TEST) activeTests++;
    if (VISUAL_TEST) activeTests++;
    
    if (activeTests == 0) {
        Serial.println("ERROR: No test selected. Enable exactly one test.");
        return false;
    } else if (activeTests > 1) {
        Serial.println("ERROR: Multiple tests selected. Enable exactly one test.");
        return false;
    }
    
    return true;
}

int percentToDutyCycle(int percent) {
    return map(percent, 0, 100, 0, 255);
}

// MOTOR TEST FUNCTIONS

void setupMotorTest() {
    Serial.println("=MOTOR TEST SETUP=");
    Serial.println("Testing both motors with 10kHz PWM");
    Serial.println("Duty cycle progression: 10% -> 20% -> 30% -> 40% -> 50% -> 60% -> 70% -> 75% -> 0%");
    Serial.println("Duration: 5 seconds per step");
    Serial.println();
    
    // Configure motor PWM pins
    gpio->configurePWMPin(MOTOR_A_PIN, MOTOR_FREQUENCY, 8);
    gpio->configurePWMPin(MOTOR_B_PIN, MOTOR_FREQUENCY, 8);
    
    // Start with motors off
    gpio->writePWM(MOTOR_A_PIN, 0);
    gpio->writePWM(MOTOR_B_PIN, 0);
    
    currentTestStep = 0;
    lastTestTime = millis();
    
    Serial.println("Motor test starting in 3 seconds...");
    delay(3000);
}

void runMotorTest() {
    unsigned long currentTime = millis();
    
    // Check if it's time for the next step
    if (currentTime - lastTestTime >= TEST_DURATION_MS) {
        if (currentTestStep < NUM_DUTY_STEPS) {
            int dutyPercent = DUTY_STEPS[currentTestStep];
            int dutyCycle = percentToDutyCycle(dutyPercent);
            
            Serial.print("Step ");
            Serial.print(currentTestStep + 1);
            Serial.print("/");
            Serial.print(NUM_DUTY_STEPS);
            Serial.print(": Setting motors to ");
            Serial.print(dutyPercent);
            Serial.print("% duty cycle (");
            Serial.print(dutyCycle);
            Serial.println("/255)");
            
            // Set both motors to same duty cycle
            gpio->writePWM(MOTOR_A_PIN, dutyCycle);
            gpio->writePWM(MOTOR_B_PIN, dutyCycle);
            
            currentTestStep++;
            lastTestTime = currentTime;
        } else {
            // Test complete
            Serial.println("= MOTOR TEST COMPLETE =");
            Serial.println("Motors stopped. Test finished.");
            gpio->writePWM(MOTOR_A_PIN, 0);
            gpio->writePWM(MOTOR_B_PIN, 0);
            
            // Prevent further execution
            while(true) {
                delay(1000);
            }
        }
    }
}
// PHOTOSENSOR TEST FUNCTIONS 

void setupPhotosensorTest() {
    Serial.println("= PHOTOSENSOR TEST SETUP =");
    Serial.println("Testing all three with logic verification");
    Serial.println("Left: Pin " + String(PHOTO_SENSOR_A_PIN));
    Serial.println("Center: Pin " + String(PHOTO_SENSOR_B_PIN)); 
    Serial.println("Right: Pin " + String(PHOTO_SENSOR_C_PIN));
    Serial.println();
    Serial.println("Test Conditions:");
    Serial.println("- Logic HIGH (all 3 sensors) = WHITE surface = OFF_LINE state");
    Serial.println("- Logic LOW (any sensor) = BLACK surface = line detected");
    Serial.println("- Using LineDetector for state analysis");
    Serial.println();
    
    // Configure photosensor pins as analog inputs
    std::map<int, std::string> sensorPins = {
        {PHOTO_SENSOR_A_PIN, "analog_input"},
        {PHOTO_SENSOR_B_PIN, "analog_input"},
        {PHOTO_SENSOR_C_PIN, "analog_input"}
    };
    gpio->initializePins(sensorPins);
    // Create PhotoSensor instances
    testSensorLeft = new PhotoSensor(PHOTO_SENSOR_A_PIN, 512);   // Pin 3 (left)
    testSensorCenter = new PhotoSensor(PHOTO_SENSOR_B_PIN, 512); // Pin 2 (center)
    testSensorRight = new PhotoSensor(PHOTO_SENSOR_C_PIN, 512);  // Pin 1 (right)
    
    // Initialize PhotoSensors
    testSensorLeft->initialize();
    testSensorCenter->initialize();
    testSensorRight->initialize();
    
    // Create LineDetector with the three sensors
    testLineDetector = new LineDetector(*testSensorLeft, *testSensorCenter, *testSensorRight, 400, 600);
    
    lastTestTime = millis();
    Serial.println("Photosensor test starting...");
    Serial.println("Expected: When all 3 sensors HIGH -> LineDetector should report OFF_LINE");
    Serial.println();
}

void runPhotosensorTest() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastTestTime >= SENSOR_READ_INTERVAL_MS) {
        // Get LineDetector's analysis
        LineState lineState = testLineDetector->detectLineState();
        float linePosition = testLineDetector->calculateLinePosition();
        
        // Get raw readings directly from PhotoSensors
        int leftReading = testSensorLeft->readRaw();
        int centerReading = testSensorCenter->readRaw();
        int rightReading = testSensorRight->readRaw();

        // Display raw readings
        Serial.print("Raw Sensors - Left: ");
        Serial.print(leftReading);
        Serial.print(", Center: ");
        Serial.print(centerReading);
        Serial.print(", Right: ");
        Serial.print(rightReading);
        
        // Simple threshold check (using same logic as LineDetector would)
        int threshold = 512; // Use same threshold as PhotoSensor constructor
        bool leftBlack = leftReading < threshold;
        bool centerBlack = centerReading < threshold;
        bool rightBlack = rightReading < threshold;
        
        // Display sensor states
        Serial.print(" | States - L:");
        Serial.print(leftBlack ? "BLACK" : "WHITE");
        Serial.print(", C:");
        Serial.print(centerBlack ? "BLACK" : "WHITE");
        Serial.print(", R:");
        Serial.print(rightBlack ? "BLACK" : "WHITE");
        
        // Display LineDetector analysis
        Serial.print(" | LineState: ");
        switch(lineState) {
            case LineState::ON_LINE:
                Serial.print("ON_LINE -> FORWARD");
                break;
            case LineState::TURN_LEFT:
                Serial.print("TURN_LEFT -> TURN LEFT");
                break;
            case LineState::TURN_RIGHT:
                Serial.print("TURN_RIGHT -> TURN RIGHT");
                break;
            case LineState::OFF_LINE:
                Serial.print("OFF_LINE -> STOP");
                break;
            case LineState::UNKNOWN:
                Serial.print("UNKNOWN -> ASSESS");
                break;
        }
        
        Serial.print(" | Position: ");
        Serial.println(linePosition, 3);

        // Special highlighting for the test condition
        bool allWhite = !leftBlack && !centerBlack && !rightBlack;
                       
        if (allWhite && lineState == LineState::OFF_LINE) {
            Serial.println(">>> TEST CONDITION MET: All 3 sensors WHITE -> LineDetector reports OFF_LINE <<<");
            Serial.println(">>> EXPECTED OUTPUT: System should STOP (no line to follow) <<<");
            Serial.println(">>> LOGIC VERIFICATION: PASS ✓ <<<");
        }
        
        lastTestTime = currentTime;
    }
}

// BUTTON TEST FUNCTIONS

void setupButtonTest() {
    Serial.println("== BUTTON TEST SETUP =");
    Serial.println("Testing user button on Pin " + String(USER_BUTTON_PIN));
    Serial.println("Press and release button to see state changes");
    Serial.println("Button readings every 100ms with debouncing");
    Serial.println();
    
    // Configure button pin with pullup
    gpio->configurePin(USER_BUTTON_PIN, "digital_input_pullup");
    
    lastButtonState = gpio->readDigital(USER_BUTTON_PIN);
    lastTestTime = millis();
    lastButtonTime = millis();
    
    Serial.println("Button test starting... (Press Ctrl+C to stop)");
}

void runButtonTest() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastTestTime >= BUTTON_READ_INTERVAL_MS) {
        bool currentButtonState = gpio->readDigital(USER_BUTTON_PIN);
        
        // Debouncing
        if (currentButtonState != lastButtonState) {
            if (currentTime - lastButtonTime >= BUTTON_DEBOUNCE_MS) {
                Serial.print("Button state changed: ");
                Serial.println(currentButtonState ? "RELEASED" : "PRESSED");
                
                lastButtonState = currentButtonState;
                lastButtonTime = currentTime;
            }
        }
        
        // Periodic status (every 5 seconds when no changes)
        static unsigned long lastStatus = 0;
        if (currentTime - lastStatus >= 5000) {
            Serial.print("Button status: ");
            Serial.println(currentButtonState ? "UP" : "DOWN");
            lastStatus = currentTime;
        }
        
        lastTestTime = currentTime;
    }
}


void setupPowerTest() {
    // Turn on the built-in LED immediately
    #ifdef LED_BUILTIN
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, HIGH);
    #else
        // ESP32-C6 DevKit typically uses GPIO8
        const int ONBOARD_LED_PIN = 8;
        pinMode(ONBOARD_LED_PIN, OUTPUT);
        digitalWrite(ONBOARD_LED_PIN, HIGH);
    #endif

    lastTestTime = millis();
    currentTestStep = 0; // Use this to track blink state

}

void runPowerTest() {
    unsigned long currentTime = millis();
    
    // Blink for 5 seconds total
    if (currentTime - lastTestTime < 5000) {
        // Blink every 500ms (on for 250ms, off for 250ms)
        bool ledState = ((currentTime / 250) % 2) == 0;
        
        #ifdef LED_BUILTIN
            digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
        #else
            const int ONBOARD_LED_PIN = 8;
            digitalWrite(ONBOARD_LED_PIN, ledState ? HIGH : LOW);
        #endif
    } else {
        // After 5 seconds, turn LED off permanently
        #ifdef LED_BUILTIN
            digitalWrite(LED_BUILTIN, LOW);
        #else
            const int ONBOARD_LED_PIN = 8;
            digitalWrite(ONBOARD_LED_PIN, LOW);
        #endif
        
    }
    
    delay(50); // Small delay for stable blinking
}


// PLACEHOLDER TEST FUNCTIONS (TBD)

void setupAudioTest() {
    Serial.println("=UDIO TEST =");
    Serial.println("TBD - Audio test not implemented yet");
    while(true) delay(1000);
}

void setupVisualTest() {
    Serial.println("=VISUAL TEST ===");
    Serial.println("TBD - Visual test not implemented yet");
    while(true) delay(1000);
}

// MAIN SETUP AND LOOP
void setup() {
    // Initialize serial with proper ESP32-C6 USB CDC handling
    Serial.begin(9600);  // Changed from 115200 to match monitor_speed
    
    // Wait for USB serial connection (important for ESP32-C6 native USB)
    while (!Serial && millis() < 5000) {
        delay(100);
    }
    delay(1000);
    
    Serial.println("BuzzCar Hardware Integration Test");
    Serial.flush();
    
    // Validate test configuration
    if (!validateTestSwitches()) {
        while(true) {
            delay(1000);
            Serial.println("Fix test configuration and restart!");
        }
    }
    
    // Set CPU frequency for consistent timing
    setCpuFrequencyMhz(160);
    Serial.print("CPU Frequency: ");
    Serial.print(getCpuFrequencyMhz());
    Serial.println(" MHz");
    Serial.println();
    
    // Get GPIO manager instance
    gpio = &GPIOManager::getInstance();
    
    // Run the selected test setup
    if (MOTOR_TEST) {
        setupMotorTest();
    } else if (PHOTOSENSOR_TEST) {
        setupPhotosensorTest();
    } else if (BUTTON_TEST) {
        setupButtonTest();
    } else if (POWER_TEST) {
        setupPowerTest();
    } else if (AUDIO_TEST) {
        setupAudioTest();
    } else if (VISUAL_TEST) {
        setupVisualTest();
    }
}

void loop() {
    // Run the selected test
    if (MOTOR_TEST) {
        runMotorTest();
    } else if (PHOTOSENSOR_TEST) {
        runPhotosensorTest();
    } else if (BUTTON_TEST) {
        runButtonTest();
    } else if (POWER_TEST) {
        runPowerTest();
    }
    // Power, Audio, and Visual tests don't have loop functions yet (TBD)
    
    delay(10); // Small delay to prevent watchdog issues
}