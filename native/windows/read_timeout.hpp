#pragma once
#include <cstdint>
#include <cstddef>

// SERIAL_TIMEOUTS semantics; clocks and lengths are explicit for deterministic tests.
inline bool read_finished(size_t done,size_t capacity,uint64_t elapsed,uint64_t idle,
                          uint32_t interval,uint32_t multiplier,uint32_t constant) {
    if(done==capacity)return true;
    if(interval==UINT32_MAX && !multiplier && !constant)return true;
    if(interval==UINT32_MAX && multiplier==UINT32_MAX && constant<UINT32_MAX)
        return done!=0 || elapsed>=constant;
    const uint64_t total=uint64_t(multiplier)*capacity+constant;
    return (total && elapsed>=total) ||
           (done && interval && interval!=UINT32_MAX && idle>=interval);
}
