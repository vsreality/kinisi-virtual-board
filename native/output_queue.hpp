#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include <stdexcept>

class OutputQueue {
    std::vector<uint8_t> bytes_;
    uint64_t last_progress_=0;
    bool waiting_=false,was_lost_=false;
public:
    void session(bool lost,bool disconnected=false) {
        if(disconnected || (lost && !was_lost_)){bytes_.clear();waiting_=false;}
        was_lost_=lost;
    }
    void append(const uint8_t* bytes,size_t count) {
        if(count>65536-bytes_.size())throw std::runtime_error("Client is not consuming output");
        if(count)bytes_.insert(bytes_.end(),bytes,bytes+count);
    }
    template<class Writer> void flush(uint64_t now,Writer write) {
        if(bytes_.empty())return;
        if(!waiting_){last_progress_=now;waiting_=true;}
        size_t sent=write(bytes_.data(),bytes_.size());
        if(sent>bytes_.size())throw std::runtime_error("Invalid endpoint write result");
        bytes_.erase(bytes_.begin(),bytes_.begin()+sent);
        if(sent){last_progress_=now;waiting_=!bytes_.empty();}
        else if(now-last_progress_>=1000000)throw std::runtime_error("Client output stalled for one second");
    }
};
