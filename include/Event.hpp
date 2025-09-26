#pragma once
#include <string>

// Event types of the Buzzcar FSM system
enum class EventType {
    // User Interface Events
    START,           // Button pressed to start system
    STOP,            // Button pressed to stop or off-line detected

    // Line Following Events
    TURN_LEFT,       // LineDetector detected need to turn left
    TURN_RIGHT,      // LineDetector detected need to turn right
    FORWARD,         // LineDetector detected need to move forward


    // System State Events
    START_MOVEMENT,   // Transition from IdleState to ForwardState
    OFF_LINE,           // LineDetector lost the line

    NONE,
    UNKNOWN          // Default/error state
};

// Event class - carries event type
class Event {
    private:
        EventType type;
        int data; // Optional data field
        unsigned long timestamp;
    public:
        // Constructors
        Event();
        Event(EventType eventType);
        Event(EventType eventType, int eventData);

        // Getters
        EventType getType() const;
        int getData() const;
        unsigned long getTimestamp() const;

        // Utility methods
        bool isValid() const;
        const char* toString() const;

        // Comparison operators
        bool operator==(const Event& other) const;
        bool operator!=(const Event& other) const;
};
