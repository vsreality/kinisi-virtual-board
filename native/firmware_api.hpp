#pragma once
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <dlfcn.h>
#endif

// One library per executable, one board per process. No Python runtime.
class Firmware {
    void* handle_ = nullptr;
    template<class T> T symbol(const char* name) {
#ifdef _WIN32
        auto value = reinterpret_cast<T>(GetProcAddress(static_cast<HMODULE>(handle_), name));
#else
        auto value = reinterpret_cast<T>(dlsym(handle_, name));
#endif
        if (!value) throw std::runtime_error(std::string("Missing firmware API: ") + name);
        return value;
    }
public:
    void (*init)();
    int (*feed)(const uint8_t*, size_t);
    int (*poll)(uint64_t);
    size_t (*read)(uint8_t*, size_t);
    uint64_t (*deadline)();
    void (*disconnect)();
    int (*lost)();
    double (*pwm)(uint8_t);
    double (*resolution)(uint8_t);
    int (*reverse)(uint8_t);
    int (*mode)(uint8_t);
    void (*encoder)(uint8_t, uint16_t);
    void (*shaft_velocity)(const double*);
    int (*imu_mount)(const double*, const double*);
    explicit Firmware(const std::filesystem::path& file) {
#ifdef _WIN32
        handle_ = LoadLibraryExW(std::filesystem::absolute(file).c_str(), nullptr,
                                 LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
#else
        handle_ = dlopen(std::filesystem::absolute(file).c_str(), RTLD_NOW | RTLD_LOCAL);
#endif
        if (!handle_) throw std::runtime_error("Cannot load firmware library: " + file.string());
        try {
#define LOAD(name) name = symbol<decltype(name)>("sim_" #name)
            LOAD(init); LOAD(feed); LOAD(poll); LOAD(read); LOAD(deadline); LOAD(disconnect);
            LOAD(lost); LOAD(pwm); LOAD(resolution); LOAD(reverse); LOAD(mode); LOAD(encoder);
            LOAD(shaft_velocity);
            LOAD(imu_mount);
#undef LOAD
            init();
        } catch (...) { unload(); throw; }
    }
    Firmware(const Firmware&) = delete;
    Firmware& operator=(const Firmware&) = delete;
    void unload() {
        if (!handle_) return;
#ifdef _WIN32
        FreeLibrary(static_cast<HMODULE>(handle_));
#else
        dlclose(handle_);
#endif
        handle_ = nullptr;
    }
    ~Firmware() { if (handle_) disconnect(); unload(); }
};
