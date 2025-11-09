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