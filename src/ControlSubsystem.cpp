#include "ControlSubsystem.hpp"
#include <Arduino.h>
#include "FSM.hpp"
#include "IdleState.hpp"
#include "ForwardState.hpp" 
#include "TurnLeftState.hpp"
#include "TurnRightState.hpp"
#include "StopState.hpp"
#include "GPIOManager.hpp"


ControlSubsystem::ControlSubsystem()
    : motorA(nullptr), motorB(nullptr), lineDetector(nullptr), pidController(nullptr) {
        // Create FSM with this as context
        fsm = std::make_unique<FSM>(this);

        // Initialize state pointers to nullptr -- they should be created in initialize()
        idleState = nullptr;
        forwardState = nullptr;
        turnLeftState = nullptr;
        turnRightState = nullptr;
        stopState = nullptr;
}

ControlSubsystem::~ControlSubsystem() {
    // FSM will handle cleanup of states internally
}

void ControlSubsystem::initialize() {
    // Initialization code

    // hardware initialization
    
    GPIOManager& gpio = GPIOManager::getInstance();
    std::map<int, std::string> pinMappings = {
        // Inputs
        {PHOTO_SENSOR_A_PIN, "analog_input"}, // Pin 3
        {PHOTO_SENSOR_B_PIN, "analog_input"}, // Pin 2
        {PHOTO_SENSOR_C_PIN, "analog_input"}, // Pin 1
        {USER_BUTTON_PIN, "digital_input_pullup"}, // Pin 11

        // Outputs
        {MOTOR_A_PIN, "pwm_output"}, // Pin 20
        {MOTOR_B_PIN, "pwm_output"}, // Pin 19
        {AUDIO_PIN, "pwm_output"},    // Pin 23
        {LCD_DATA_PIN, "digital_output"}, // Pin 22
        {LCD_CLK_PIN, "digital_output"}    // Pin 21

    };

    gpio.initializePins(pinMappings);

    // Initialize PhotoSensors
    sensorLeft = new PhotoSensor(PHOTO_SENSOR_A_PIN);
    sensorCenter = new PhotoSensor(PHOTO_SENSOR_B_PIN);
    sensorRight = new PhotoSensor(PHOTO_SENSOR_C_PIN);

    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();

    // Initialize LineDetector with the three sensors
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight);

    // Initialize Motors
    motorA = new Motor(MOTOR_A_PIN);
    motorB = new Motor(MOTOR_B_PIN);

    motorA->initialize();
    motorB->initialize();

    // Set motor parameters for line following
    motorA->setMinimumStartPWM(20); // Example value
    motorB->setMinimumStartPWM(20); // Example value

    // Initialize PID Controller
    pidController = new PIDController(2.0f, 0.1f, 0.5f);
    pidController->setOutputLimits(-40.0f, 40.0f); // TODO: Tune these limits

    // Create states
    idleState = new IdleState();
    forwardState = new ForwardState();
    turnLeftState = new TurnLeftState();
    turnRightState = new TurnRightState();
    stopState = new StopState();

    // Initialize FSM
    fsm->initialize();

    // Set initial state to idle
    fsm->transitionTo(idleState);

    Serial.println("ControlSubsystem hardware initialized.");
    Serial.println("FSM initialized to IdleState.");
    Serial.println("Motors: A(Pin 20), B(Pin 19)");
    Serial.println("PhotoSensors: Left(Pin 3), Center(Pin 2), Right(Pin 1)");
    
}

void ControlSubsystem::update() {
    // Read sensors, process events, update FSM
    fsm->update();

    Event event = generateEvent(); // Based on sensor readings
    if (event.getType() != EventType::NONE) {
        fsm->handleEvent(event);
    }
}


Event ControlSubsystem::handleEvent() {
    // Handle the event and return result
    return generateEvent();
}

Event ControlSubsystem::generateEvent() {
    // If hardware is not inititialized yet, return no event
    if (lineDetector == nullptr) {
        return Event(EventType::NONE);
    }

    // Use LineDetector's states
    LineState lineState = lineDetector->detectLineState();

    const char* currentStateName = fsm->getCurrentStateName();

    
    if(strcmp(currentStateName, "IdleState") == 0) {
        // In IdleState, start movement if line detected
        if (lineState == LineState::ON_LINE ||
            lineState == LineState::TURN_LEFT ||
            lineState == LineState::TURN_RIGHT) {
            Serial.println("Event: Line detected while idle - START_MOVEMENT");
            return Event(EventType::START_MOVEMENT);
        }
    } else if (strcmp(currentStateName, "ForwardState") == 0) {
        // In ForwardState, check for turn conditions or off-line
        if (lineState == LineState::TURN_LEFT) {
            Serial.println("Event: TURN_LEFT detected");
            return Event(EventType::TURN_LEFT);
        } else if (lineState == LineState::TURN_RIGHT) {
            Serial.println("Event: TURN_RIGHT detected");
            return Event(EventType::TURN_RIGHT);
        } else if (lineState == LineState::OFF_LINE) {
            Serial.println("Event: OFF_LINE detected");
            return Event(EventType::OFF_LINE);
        }
    } else if (strcmp(currentStateName, "TurnLeftState") == 0) {
        // In turn left state, check if turn is complete
        if (lineState == LineState::ON_LINE) {
            Serial.println("Event: ON_LINE detected - complete left turn");
            return Event(EventType::FORWARD);
        } else if (lineState == LineState::OFF_LINE) {
            Serial.println("Event: OFF_LINE detected during left turn");
            return Event(EventType::OFF_LINE);
        }
    } else if (strcmp(currentStateName, "TurnRightState") == 0) {
        // In turn right state, check if turn is complete
        if (lineState == LineState::ON_LINE) {
            Serial.println("Event: ON_LINE detected - complete right turn");
            return Event(EventType::FORWARD);
        } else if (lineState == LineState::OFF_LINE) {
            Serial.println("Event: OFF_LINE detected during right turn");
            return Event(EventType::OFF_LINE);
        }
    } else if (strcmp(currentStateName, "StopState") == 0) {
        // In StopState, wait for manual reset (not implemented here)
        if (lineState == LineState::ON_LINE) {
            Serial.println("Event: ON_LINE detected - reset from stop");
            return Event(EventType::START_MOVEMENT);
        }
    }

    if (lineState == LineState::UNKNOWN) {
        Serial.println("Event: UNKNOWN line state detected");
        return Event(EventType::UNKNOWN);
    }

    // No significant event detected
    return Event(EventType::NONE);
}


