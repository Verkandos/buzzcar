#include <Arduino.h>
#include <FSM.hpp>
#include "ControlSubsystem.hpp"
#include "IdleState.hpp"
#include "ForwardState.hpp"
#include "TurnLeftState.hpp"
#include "TurnRightState.hpp"
#include "StopState.hpp"
#include "Speaker.h"
#include "Screen.h"
#include "ControlConfig.hpp"

/**
 * @brief Constructs a new FSM object
 * 
 * Initializes the finite state machine with a reference to the control subsystem.
 * The FSM starts without a current state - initialize() must be called to set 
 * the initial state.
 */
FSM::FSM(ControlSubsystem* controlContext) : context(controlContext), currentState(nullptr) {
    // Initialize state instances
}

/**
 * @brief Destroys the FSM object
 * 
 * Cleanup is handled automatically by smart pointers.
 * Current state's onExit is called during destruction.
 */
FSM::~FSM() {
    // Smart pointers handle cleanup automatically
}

/**
 * @brief Initializes the FSM to the starting state (IdleState)
 * 
 * Sets the FSM to start in IdleState and calls the state's onEntry method.
 * This must be called before using the FSM for event handling or updates.
 */
void FSM::initialize() {
    // Start in IdleState
    currentState = std::make_unique<IdleState>();
    currentState->onEntry(context);
    
    Serial.println("FSM initialized - Starting in IdleState");
}

/**
 * @brief Updates the current state
 * 
 * Calls the current state's onUpdate method to perform state-specific
 * processing. Should be called regularly in the main loop.
 */
void FSM::update() {
    if (currentState) {
        currentState->onUpdate(context);
    }
}

/**
 * @brief Handles incoming events and perform state transitions
 * 
 * Processes events and triggers appropriate state transitions based on
 * the current state and event type. Implements the state machine logic
 * for line-following car behavior.
 * 
 * @param event The event to process
 * 
 * @note Valid transitions:    
 *       - START: StopState -> IdleState
 *       - STOP: Any state -> StopState (emergency)
 *       - START_MOVEMENT: IdleState -> ForwardState
 *       - TURN_LEFT/RIGHT: ForwardState -> TurnLeftState/TurnRightState
 *       - FORWARD: TurnLeftState/TurnRightState -> ForwardState
 *       - OFF_LINE: Active states -> StopState
 */
void FSM::handleEvent(const Event& event) {
    switch(event.getType()) {
        case EventType::START:
            if (getCurrentStateName() == "StopState") {
                transitionTo(new IdleState());
            }
            break;
        
        case EventType::STOP:
            transitionTo(new StopState());
            break;
        
        case EventType::START_MOVEMENT:
            if (getCurrentStateName() == "ForwardState") {
                transitionTo(new ForwardState());
            }
            break;

        case EventType::TURN_RIGHT:
            if (getCurrentStateName() == "ForwardState") {
                transitionTo(new TurnRightState());
            }
            break;
            
        case EventType::FORWARD:
            if (getCurrentStateName() == "TurnLeftState" || 
                getCurrentStateName() == "TurnRightState") {
                transitionTo(new ForwardState());
            }
            break;
        
        case EventType::OFF_LINE:
            if (getCurrentStateName() != "StopState" && 
                getCurrentStateName() != "IdleState") {
                transitionTo(new StopState());
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief Transition from current state to a new state
 * 
 * Performs a complete state transition by calling onExit() on the current state,
 * switching to the new state, and calling onEntry() to the new state.
 * 
 * @param newState Pointer to the new state to transition to (takes ownership)
 */
void FSM::transitionTo(State* newState) {
    ControlConfig& config = ControlConfig::getInstance();
    // Exit current state
    if (currentState) {
        Serial.print("FSM: Exiting ");
        Serial.println(getCurrentStateName());
        currentState->onExit(context);
        currentState.reset(); // Clear current state
    }

    // Transition to new state
    currentState.reset(newState);
    currentState->onEntry(context);

    // Provide audio/visual feedback for new state
    if (config.feedback.enableAudio || config.feedback.enableDisplay) {
        const char* stateName = getCurrentStateName();
        int direction = -1;

        if (strcmp(stateName, "StopState") == 0) direction = 0; // STOP
        else if (strcmp(stateName, "ForwardState") == 0) direction = 1; // FORWARD
        else if (strcmp(stateName, "TurnLeftState") == 0) direction = 2; // LEFT
        else if (strcmp(stateName, "TurnRightState") == 0) direction = 3; // RIGHT

        if (direction >= 0) {
            startMelodyForDirection(direction); // Audio feedback
            showDirection(direction);      // Visual feedback
            Serial.print("Feedback: Playing audio/visual for direction ");
            Serial.println(direction);
        }
    }
}



/**
 * @brief Check if a transition is valid for the given event type
 * 
 * Validates whether the current state can handle the specified event type
 * according to the state machine's transition rules.
 * 
 * @param eventType The event type to validate
 * @return true if the transition is valid, false otherwise
 * 
 * @note Transition rules:
 *       - STOP is always valid (emergency override)
 *       - START only valid from StopState
 *       - START_MOVEMENT only valid from IdleState
 *       - TURN_LEFT/RIGHT only valid from ForwardState
 *       - FORWARD only valid from TurnLeftState or TurnRightState
 *       - OFF_LINE valid from any active state (not Stop/Idle)
 */
bool FSM::isValidTransition(EventType eventType) const {
    switch (eventType) {
        case EventType::STOP:
            return true; // STOP is always valid (emergency)
        
        case EventType::START:
            return isInState("StopState");

        case EventType::START_MOVEMENT:
            return isInState("IdleState");
        
        case EventType::TURN_LEFT:
        case EventType::TURN_RIGHT:
            return isInState("ForwardState");

        case EventType::FORWARD:
            return isInState("TurnLeftState") || isInState("TurnRightState");
        
        case EventType::OFF_LINE:
            return !isInState("StopState") && !isInState("IdleState");

        default:
            return false;
    }
}
/**
 * @brief Get pointer to the current state
 * 
 * Returns a raw pointer to the current state object. Useful for accessing 
 * state-specific methods or properties.
 * 
 * @return State* Pointer to the current state (nullptr if no state)
 */
State* FSM::getCurrentState() const {
    return currentState.get();
}

/**
 * @brief Get the name of the current state
 * 
 * Returns the string name of the current state for logging or debugging.
 * 
 * @return const char* Name of the current state, or "NULL" if no state
 */
const char* FSM::getCurrentStateName() const {
    if (!currentState) {
        return "NULL";
    }
    return currentState->getName().c_str();
}

/**
 * @brief Resets the FSM to initial state (IdleState)
 * 
 * Exits the current state and resets the FSM back to the IdleState.
 * Useful for error recovery or restarting the car's behavior.
 */
void FSM::reset() {
    if (currentState) {
        currentState->onExit(context);
    }

    // Reset to IdleState
    currentState = std::make_unique<IdleState>();
    currentState->onEntry(context);

    Serial.println("FSM has been reset to IdleState.");
}

/**
 * @brief Check if FSM is currently in the specified state
 * 
 * Compares the current name with the provided state name string..
 * Case-sensitive string comparison.
 * 
 * @param stateName The name of the state to check against
 * @return true if the FSM is in the specified state, false otherwise
 */
bool FSM::isInState(const char* stateName) const {
    return strcmp(getCurrentStateName(), stateName) == 0;
}