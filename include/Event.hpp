#pragma once
#include <string>

enum class EventType {
    START,           // Button pressed to start system
    STOP,            // Button pressed to stop or off-line detected
    TURN_LEFT,       // Line detector says turn left
    TURN_RIGHT,      // Line detector says turn right
    FORWARD,         // Return to forward movement
    // ... other events
};

class Event {
    private:
        EventType eventType;
        std::string source;
        int timestamp;
    public:
        Event() = default;
        Event(EventType eventType, const std::string& source, int timestamp)
            : eventType(eventType), source(source), timestamp(timestamp) {}
        EventType getType() const;
        std::string getSource() const;
        int getTimestamp() const;
};
