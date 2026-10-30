#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <algorithm>

// All calls are serialized by the device's WDF wait lock.
template<size_t Capacity> struct Ring {
    uint8_t bytes[Capacity];
    size_t head,size;
    void clear(){head=size=0;}
    bool push(const uint8_t* input,size_t length) {
        if(length>Capacity-size)return false;
        const size_t tail=(head+size)%Capacity;
        const size_t first=std::min(length,Capacity-tail);
        if(first)std::memcpy(bytes+tail,input,first);
        if(length>first)std::memcpy(bytes,input+first,length-first);
        size+=length;return true;
    }
    size_t pop(uint8_t* output,size_t capacity) {
        const size_t length=std::min(size,capacity),first=std::min(length,Capacity-head);
        if(first)std::memcpy(output,bytes+head,first);
        if(length>first)std::memcpy(output+first,bytes,length-first);
        head=(head+length)%Capacity;size-=length;return length;
    }
};
