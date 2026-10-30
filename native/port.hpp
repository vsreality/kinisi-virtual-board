#pragma once
#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

struct Port {
    std::string name;
    virtual ~Port() = default;
    // Exchange also observes disconnect/reconnect generations on Windows.
    virtual size_t read(uint8_t* buffer, size_t capacity) = 0;
    virtual size_t write(const uint8_t* buffer, size_t length) = 0;
    virtual bool take_disconnect() { return false; }
};
std::unique_ptr<Port> open_port();
