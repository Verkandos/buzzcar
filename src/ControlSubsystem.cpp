#include "ControlSubsystem.hpp"
#include <Arduino.h>
#include "FSM.hpp"
#include "IdleState.hpp"
#include "ForwardState.hpp" 
#include "TurnLeftState.hpp"
#include "TurnRightState.hpp"
#include "StopState.hpp"


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

    // motorA = new Motor(MOTOR_A_PIN);
    // motorB = new Motor(MOTOR_B_PIN);
    // lineDetector = new LineDetector(LINE_SENSOR_LEFT_PIN, LINE_SENSOR_RIGHT_PIN);
    // pidController = new PIDController(1.0, 0.0, 0.0); // Default PID gains
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



