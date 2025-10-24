#include "ControlSubsystem.hpp"
#include <Arduino.h>
#include "FSM.hpp"
#include "IdleState.hpp"
#include "ForwardState.hpp" 
#include "TurnLeftState.hpp"
#include "TurnRightState.hpp"
#include "StopState.hpp"
#include "GPIOManager.hpp"
#include "Speaker.h"
#include "Screen.h"
#include "ControlConfig.hpp"

/**
 * @brief Constructs a new ControlSubsystem instance
 * 
 * Initializes all hardware component pointers to nullptr and creates
 * the FSM (Finite State Machine) with this subsystem as context.
 * Actual hardware initialization is done in the initialize() method
 */
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
/**
 * @brief Destroys the ControlSubsystem instance
 * 
 * Cleans up all dynamically allocated hardware components including
 * motors, sensors, line detector, and PID controller. FSM-managed
 * states are automatically cleaned up by the FSM destructor.
 */
ControlSubsystem::~ControlSubsystem() {
    // Clean up dynamically allocated resources
    delete motorA;
    delete motorB;
    delete lineDetector;
    delete sensorLeft;
    delete sensorCenter;
    delete sensorRight;
    delete pidController;
    // States are managed by FSM, so no need to delete them here
}

/**
 * @brief Initializes all hardware components and the FSM
 * 
 * Performs complete system initialization including:
 * - GPIO pin configuration for motors, sensors, audio, and display
 * - PhotoSensor and LineDetector setup for line detection
 * - Motor configuration
 * - Screen and Speaker system initialization
 * - PID Controller setup for smooth line following
 * - FSM state creation and initialization (tranistion to IdleState)
 * 
 * @note This method must be called before using any ControlSubsytem functionality
 * @note  Pin assignments: Motors (19, 20), Sensors (1, 2, 3), Audio (23), LCD (21, 22), Button (11)
 */
void ControlSubsystem::initialize() {
    // Initialization code
    ControlConfig& config = ControlConfig::getInstance();


    // hardware initialization
    
    GPIOManager& gpio = GPIOManager::getInstance();
    std::map<int, std::string> pinMappings = {
        // Inputs
        {config.pins.photoSensorA, "analog_input"}, // Pin 3
        {config.pins.photoSensorB, "analog_input"}, // Pin 2
        {config.pins.photoSensorC, "analog_input"}, // Pin 1
        {config.pins.userButton, "digital_input_pullup"}, // Pin 11

        // Outputs
        {config.pins.motorA, "pwm_output"}, // Pin 20
        {config.pins.motorB, "pwm_output"}, // Pin 19
        {config.pins.audio, "pwm_output"},    // Pin 23
        {config.pins.lcdData, "digital_output"}, // Pin 22
        {config.pins.lcdClk, "digital_output"}    // Pin 21

    };

    gpio.initializePins(pinMappings);

    // Initialize PhotoSensors
    sensorLeft = new PhotoSensor(config.pins.photoSensorA);
    sensorCenter = new PhotoSensor(config.pins.photoSensorB);
    sensorRight = new PhotoSensor(config.pins.photoSensorC);

    sensorLeft->initialize();
    sensorCenter->initialize();
    sensorRight->initialize();

    // Initialize LineDetector with the three sensors
    lineDetector = new LineDetector(*sensorLeft, *sensorCenter, *sensorRight, config.sensors.blackThreshold,config.sensors.whiteThreshold);

    // Initialize Motors
    motorA = new Motor(config.pins.motorA);
    motorB = new Motor(config.pins.motorB);

    motorA->initialize();
    motorB->initialize();

    // Set motor parameters for line following
    motorA->setMinimumStartPWM(config.motor.minStartPWM);
    motorB->setMinimumStartPWM(config.motor.minStartPWM);

    motorA->setPWMRange(config.motor.minStartPWM, config.motor.maxPWM);
    motorB->setPWMRange(config.motor.minStartPWM, config.motor.maxPWM);

    // Initialize Screen and Speaker
    screenBegin(config.pins.lcdData, config.pins.lcdClk);
    speakerBegin(config.pins.audio, 2, 10, config.feedback.audioVolume); // Use config volume

    // Show initial state on screen
    showDirection(0); // Show STOP initially

    // Initialize PID Controller
    pidController = new PIDController(config.pid.Kp, config.pid.Ki, config.pid.Kd);
    pidController->setOutputLimits(config.pid.outputMin, config.pid.outputMax); // TODO: Tune these limits
    pidController->setIntegralLimit(config.pid.integralLimit);

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
    Serial.printf("Motors: A(Pin %d), B(Pin %d)\n", config.pins.motorA, config.pins.motorB);
    Serial.printf("PhotoSensors: Left(Pin %d), Center(Pin %d), Right(Pin %d)\n", 
                  config.pins.photoSensorA, config.pins.photoSensorB, config.pins.photoSensorC);
    
}

/**
 * @brief Main update loop for the ControlSubsystem
 * 
 * Performs the following operations in each cycle:
 * 1. Services the speaker to maintain continuous audio playback
 * 2. Updates the FSM to process current state logic
 * 3. Generates events based on current sensor readings
 * 4. Handles generated events through the FSM for state transitions
 * 
 * @note This method should be called continuously in the main loop
 * @note Event generation and handling are decoupled for better modularity
 */
void ControlSubsystem::update() {
    // Service the speaker melody
    serviceMelody();

    // Read sensors, process events, update FSM
    fsm->update();

    Event event = generateEvent(); // Based on sensor readings
    if (event.getType() != EventType::NONE) {
        fsm->handleEvent(event);
    }
}

/**
 * @brief Handles events by delegating to generateEvent()
 * 
 * This method serves as a wrapper around generateEvent() for compatibility
 * with the existing event handling interfrace.
 * 
 * @return Event The generated event based on current system state
 * 
 */
Event ControlSubsystem::handleEvent() {
    // Handle the event and return result
    return generateEvent();
}
/**
 * @brief Generates events based on current sensor readings and car state
 * 
 * Analyzes the current line detection state and car FSM state to determine
 * appropriate events for state transitions. Events generation follows these rules:
 * 
 * - From IdleState: START_MOVEMENT when line is detected
 * - From ForwardState: TURN_LEFT/TURN_RIGHT when turn conditions detected, OFF_LINE when line lost
 * - From TurnLeftState/TurnRightState: FORWARD when line is regained, OFF_LINE if line lost
 * - From StopState: START_MOVEMENT when line is detected (manual reset)
 * 
 * @return Event The appropriate event type based on current conditions:
 *         - START_MOVEMENT: Line detected while idle or stopped
 *         - TURN_LEFT: Left turn required while moving forward
 *         - TURN_RIGHT: Right turn required while moving forward  
 *         - FORWARD: Turn completed, resume forward motion
 *         - OFF_LINE: Line lost, stop car
 *         - UNKNOWN: Inconsistent sensor readings
 *         - NONE: No significant state change detected
 * 
 * @note Returns NONE if hardware is not yet initialized (lineDetector is nullptr)
 * @note All event decisions are logged to Serial for debugging
 * 
 */
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


