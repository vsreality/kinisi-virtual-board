#include "plant.hpp"
#include "port.hpp"
#include "output_queue.hpp"
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>
#include <sstream>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

static volatile std::sig_atomic_t stopping=0;
static void stop(int) { stopping=1; }

template<size_t N> static std::array<double, N> mount_numbers(const std::string& text) {
    std::array<double, N> values{};
    std::istringstream stream(text);
    std::string item;
    if (text.empty() || text.back() == ',') {
        throw std::runtime_error("Malformed IMU mount vector");
    }
    for (size_t i = 0; i < N; ++i) {
        if (!std::getline(stream, item, ',')) {
            throw std::runtime_error("Wrong IMU mount component count");
        }
        size_t used = 0;
        values[i] = std::stod(item, &used);
        if (used != item.size() || !std::isfinite(values[i])) {
            throw std::runtime_error("IMU mount components must be finite numbers");
        }
    }
    if (std::getline(stream, item, ',')) {
        throw std::runtime_error("Wrong IMU mount component count");
    }
    return values;
}

// Binary deterministic test interface: op:u8, length:u32 LE, payload.
// F feeds wire bytes, A advances by u64 microseconds, D disconnects, Q quits.
// Response is length:u32 LE + the unmodified firmware output.
static void write32(uint32_t n) { for(unsigned i=0;i<4;++i)std::cout.put(char(n>>(8*i))); }
static void exact(char* data,size_t n) {
    if(!std::cin.read(data,static_cast<std::streamsize>(n)))throw std::runtime_error("Truncated test input");
}
static int stdio_test(Firmware& fw,Plant& plant) {
#ifdef _WIN32
    _setmode(_fileno(stdin),_O_BINARY);_setmode(_fileno(stdout),_O_BINARY);
#endif
    for(int op; (op=std::cin.get())!=EOF;) {
        uint8_t raw[4];exact(reinterpret_cast<char*>(raw),4);
        uint32_t count=uint32_t(raw[0])|(uint32_t(raw[1])<<8)|(uint32_t(raw[2])<<16)|(uint32_t(raw[3])<<24);
        if(count>65536)throw std::runtime_error("Test packet too large");
        std::vector<uint8_t> data(count);exact(reinterpret_cast<char*>(data.data()),count);
        if(op=='F') { if(!fw.feed(data.data(),data.size()))throw std::runtime_error("Firmware RX full");plant.poll(); }
        else if(op=='A' && count==8) {
            uint64_t delta=0;for(unsigned i=0;i<8;++i)delta|=uint64_t(data[i])<<(8*i);
            if(delta>3600000000ULL)throw std::runtime_error("Test advance exceeds one hour");
            plant.advance_to(plant.now+delta);
        } else if(op=='D' && !count) fw.disconnect();
        else if(op=='Q' && !count) return 0;
        else throw std::runtime_error("Invalid test operation");
        uint8_t output[65536];size_t length=fw.read(output,sizeof(output));
        write32(static_cast<uint32_t>(length));std::cout.write(reinterpret_cast<char*>(output),length);std::cout.flush();
    }
    return 0;
}

int main(int argc,char** argv) {
    try {
#ifdef _WIN32
        wchar_t executable[32768];
        DWORD used=GetModuleFileNameW(nullptr,executable,32768);
        if(!used || used==32768)throw std::runtime_error("Cannot locate simulator executable");
        auto directory=std::filesystem::path(executable).parent_path();
        std::filesystem::path library=directory/"firmware.dll";
#else
        auto directory=std::filesystem::read_symlink("/proc/self/exe").parent_path();
        std::filesystem::path library=directory/"firmware.so";
#endif
        bool test=false;
        bool mounted = false;
        std::array<double, 3> imu_offset{};
        std::array<double, 9> imu_rotation{1., 0., 0., 0., 1., 0., 0., 0., 1.};
        for(int i=1;i<argc;++i) {
            std::string option=argv[i];
            if(option=="--stdio-test")test=true;
            else if(option=="--library" && i+1<argc)library=argv[++i];
            else if (option == "--imu-offset-m" && i + 1 < argc) {
                imu_offset = mount_numbers<3>(argv[++i]);
                mounted = true;
            }
            else if (option == "--imu-board-to-chassis" && i + 1 < argc) {
                imu_rotation = mount_numbers<9>(argv[++i]);
                mounted = true;
            }
            else if(option=="--help") {
                std::cout<<"Kinisi standalone firmware simulator\n"
                  "  kinisi-sim [--library PATH]\n"
                  "Creates one Windows COM device (installed Kinisi driver required) or Linux PTY.\n"
                  "  --imu-offset-m x,y,z   chip position in chassis forward/left/up metres\n"
                  "  --imu-board-to-chassis r00,r01,...,r22   proper rotation, row-major\n"
                  "  --stdio-test   deterministic binary test interface\n";return 0;
            } else throw std::runtime_error("Unknown option: "+option);
        }
        Firmware firmware(library);Plant plant(firmware);
        if (mounted && !firmware.imu_mount(imu_offset.data(), imu_rotation.data())) {
            throw std::runtime_error("IMU mount requires an IMU board and a proper orthonormal rotation");
        }
        if(test)return stdio_test(firmware,plant);
        auto port=open_port();
        std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
        std::cout<<"{\"backend\":\"native-cpp\",\"serial_port\":\""<<port->name<<"\"}"<<std::endl;
        auto start=std::chrono::steady_clock::now();
        OutputQueue pending;
        while(!stopping) {
            auto now=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-start).count();
            plant.advance_to(static_cast<uint64_t>(now));
            uint8_t input[4096],output[65536];
            auto length=port->read(input,sizeof(input));
            bool disconnected=port->take_disconnect();
            if(disconnected)firmware.disconnect();
            // Observe watchdog expiry before a same-iteration INIT can clear it.
            pending.session(firmware.lost()!=0,disconnected);
            if(length && !firmware.feed(input,length))throw std::runtime_error("Firmware input overflow");
            plant.poll();
            pending.session(firmware.lost()!=0);
            auto count=firmware.read(output,sizeof(output));
            pending.append(output,count);
            pending.flush(plant.now,[&](const uint8_t* data,size_t size){return port->write(data,size);});
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        firmware.disconnect();return 0;
    } catch(const std::exception& error) { std::cerr<<"Kinisi simulator: "<<error.what()<<std::endl;return 1; }
}
