#include "ControlSubsystem.hpp"
#include "IdleState.hpp"
#include "ForwardState.hpp" 
#include "TurnLeftState.hpp"
#include "TurnRightState.hpp"
#include "StopState.hpp"

ControlSubsystem::~ControlSubsystem() {
    // FSM will handle cleanup of states internally
}

ControlSubsystem::ControlSubsystem()
    : currentState(nullptr), error(0.0f), Tmin(0), Tmax(0) {
    // Constructor implementation
}

void ControlSubsystem::initialize() {
    // Initialization code

    // Create states
    IdleState* idle = new IdleState();
    ForwardState* forward = new ForwardState();
    TurnLeftState* turnLeft = new TurnLeftState();
    TurnRightState* turnRight = new TurnRightState();
    StopState* stop = new StopState();


    // Add to states vector
    std::vector<State*> stateList = {idle, forward, turnLeft, turnRight, stop};
    fsm.initialize(stateList, idle); // Start in Idle state

    // Initialize other components as needed
}

void ControlSubsystem::update() {
    // Read sensors, process events, update FSM
    fsm.update();

    Event event = generateEvent(); // Based on sensor readings
    if (!event.getType().empty()) {
        fsm.transition(event);
    }
}

float ControlSubsystem::computeError() {
    // Compute and return error computation
    return error;
}

Event ControlSubsystem::generateEvent() {
    // Generate and return an event
    return Event();
}

Event ControlSubsystem::handleEvent() {
    // Handle the event and return result
    return Event();
}

State* ControlSubsystem::getCurrentState() {
    // Return the current state
    return fsm.getCurrentState();
}


