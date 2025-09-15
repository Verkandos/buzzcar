#pragma once
#include <string>

class Event {
    private:
        std::string type;
        std::string source;
        int timestamp;
    public:
        Event() = default;
        Event(const std::string& type, const std::string& source, int timestamp)
            : type(type), source(source), timestamp(timestamp) {}
        std::string getType() const;
        std::string getSource() const;
        int getTimestamp() const;
};
