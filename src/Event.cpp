#include "Arduino.h"
#include "Event.hpp"

std::string Event::getType() const {
    return type;
}
std::string Event::getSource() const {
    return source;
}
int Event::getTimestamp() const {
    return timestamp;
}