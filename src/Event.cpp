#include <Arduino.h>
#include "Event.hpp"

// Default constructor - creates UNKNOWN event
Event::Event() : type(EventType::UNKNOWN), data(0), timestamp(millis()) {}

// Constructor with event type only

Event::Event(EventType eventType) : type(eventType), data(0), timestamp(millis()) {}

// Constructor with event type and data
Event::Event(EventType eventType, int eventData) : type(eventType), data(eventData), timestamp(millis()) {}

// Getters

EventType Event::getType() const {
    return type;
}

int Event::getData() const {
    return data;
}

unsigned long Event::getTimestamp() const {
    return timestamp;
}

// Check if event is valid (not UNKNOWN)

bool Event::isValid() const {
    return type != EventType::UNKNOWN;
}

// Convert event type to string for debugging
const char* Event::toString() const {
    switch (type) {
        case EventType::START: return "START";
        case EventType::STOP: return "STOP";
        case EventType::TURN_LEFT: return "TURN_LEFT";
        case EventType::TURN_RIGHT: return "TURN_RIGHT";
        case EventType::FORWARD: return "FORWARD";
        case EventType::START_MOVEMENT: return "START_MOVEMENT";
        case EventType::OFF_LINE: return "OFF_LINE";
        default: return "UNKNOWN";
    }
}

// Comparison operators
bool Event::operator==(const Event& other) const {
    return type == other.type && data == other.data;
}

bool Event::operator!=(const Event& other) const {
    return !(*this == other);
}