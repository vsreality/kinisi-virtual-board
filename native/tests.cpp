#include "windows/ring.hpp"
#include "windows/read_timeout.hpp"
#include "output_queue.hpp"
#include <cassert>
#include <deque>
#include <random>

int main() {
    Ring<37> ring{};
    std::deque<uint8_t> reference;
    std::mt19937 random(1234);
    for(unsigned i=0;i<100000;++i) {
        uint8_t data[50];size_t size=random()%50;
        if(random()%2) {
            for(size_t j=0;j<size;++j)data[j]=static_cast<uint8_t>(random());
            bool accepted=ring.push(data,size);
            assert(accepted==(reference.size()+size<=37));
            if(accepted)reference.insert(reference.end(),data,data+size);
        } else {
            size_t actual=ring.pop(data,size);
            assert(actual==std::min(size,reference.size()));
            for(size_t j=0;j<actual;++j){assert(data[j]==reference.front());reference.pop_front();}
        }
        assert(ring.size==reference.size());
    }
    assert(!read_finished(0,10,1000000,1000000,0,0,0)); // indefinite
    assert(read_finished(10,10,0,0,0,0,0));
    assert(read_finished(0,10,0,0,UINT32_MAX,0,0)); // immediate
    assert(!read_finished(0,10,99,99,UINT32_MAX,UINT32_MAX,100));
    assert(read_finished(0,10,100,100,UINT32_MAX,UINT32_MAX,100));
    assert(read_finished(1,10,1,0,UINT32_MAX,UINT32_MAX,100));
    assert(!read_finished(0,10,24,24,0,2,5));
    assert(read_finished(0,10,25,25,0,2,5)); // total multiplier
    assert(!read_finished(0,10,100,100,20,0,0)); // no first byte
    assert(!read_finished(1,10,100,19,20,0,0));
    assert(read_finished(1,10,100,20,20,0,0)); // interval
    assert(!read_finished(0,65536,UINT32_MAX,UINT32_MAX,0,UINT32_MAX,1)); // no overflow
    OutputQueue output;
    const uint8_t stale[]={0,1},reply[]={2,3,4};
    output.append(stale,2);
    output.session(true); // discard old-session bytes exactly once
    output.append(reply,3);
    std::vector<uint8_t> received;
    output.flush(0,[](const uint8_t*,size_t){return size_t(0);});
    for(uint64_t time=100;time<400;time+=100) {
        output.session(true); // loss remains latched until next INIT
        output.flush(time,[&](const uint8_t* bytes,size_t){received.push_back(bytes[0]);return size_t(1);});
    }
    assert(received==std::vector<uint8_t>(reply,reply+3));
    output.append(reply,3);
    output.flush(500,[](const uint8_t*,size_t){return size_t(0);});
    output.session(true);
    bool stalled=false;
    try {output.flush(1000500,[](const uint8_t*,size_t){return size_t(0);});}
    catch(const std::runtime_error&){stalled=true;}
    assert(stalled);
    OutputQueue recovered;
    recovered.append(stale,2);
    recovered.session(true); // watchdog expires before processing incoming INIT
    recovered.session(false); // INIT recovers in the same main-loop iteration
    recovered.append(reply,3);
    received.clear();
    recovered.flush(100,[&](const uint8_t* bytes,size_t count){
        received.assign(bytes,bytes+count);return count;
    });
    assert(received==std::vector<uint8_t>(reply,reply+3));
}
