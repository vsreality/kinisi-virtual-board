#include "port.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

class Pty final : public Port {
    int master_=-1, slave_=-1;
public:
    Pty() {
        master_=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
        if (master_<0) throw std::runtime_error("posix_openpt failed");
        try {
            if (grantpt(master_) || unlockpt(master_)) throw std::runtime_error("Cannot initialize PTY");
            char path[256]{};
            if (ptsname_r(master_,path,sizeof(path))) throw std::runtime_error("Cannot name PTY");
            name=path;
            slave_=open(path,O_RDWR|O_NOCTTY|O_CLOEXEC);
            if (slave_<0) throw std::runtime_error("Cannot open PTY slave");
            termios attributes{};
            if (tcgetattr(slave_,&attributes)) throw std::runtime_error("Cannot read PTY settings");
            cfmakeraw(&attributes);
            if (tcsetattr(slave_,TCSANOW,&attributes)) throw std::runtime_error("Cannot set raw PTY mode");
        } catch (...) { if(slave_>=0)close(slave_);close(master_);throw; }
    }
    ~Pty() override { close(slave_);close(master_); }
    size_t read(uint8_t* data,size_t size) override {
        auto n=::read(master_,data,size);
        if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR))return 0;
        if(n<0)throw std::runtime_error(std::string("PTY read: ")+strerror(errno));
        return static_cast<size_t>(n);
    }
    size_t write(const uint8_t* data,size_t size) override {
        auto n=::write(master_,data,size);
        if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR))return 0;
        if(n<0)throw std::runtime_error(std::string("PTY write: ")+strerror(errno));
        return static_cast<size_t>(n);
    }
};
std::unique_ptr<Port> open_port() { return std::make_unique<Pty>(); }
