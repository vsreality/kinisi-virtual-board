#pragma once
#include "firmware_api.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

class Plant {
    Firmware& firmware_;
    std::array<double,4> counts_{};
public:
    uint64_t now = 0;
    double max_speed = 10, time_constant = .05;
    std::array<double,4> velocity{}, angle{};
    std::array<unsigned,4> encoder_motors{0,1,2,3};
    explicit Plant(Firmware& firmware) : firmware_(firmware) {}
    void poll() {
        int code = firmware_.poll(now);
        if (code) throw std::runtime_error("Firmware scheduler invariant: " + std::to_string(code));
    }
    void advance_to(uint64_t target) {
        if (target < now) throw std::runtime_error("Clock moved backwards");
        while (now < target) {
            auto deadline = firmware_.deadline();
            if (deadline <= now) { poll(); deadline = firmware_.deadline(); }
            if (deadline <= now) throw std::runtime_error("Firmware task did not yield");
            const uint64_t step = std::min({uint64_t(1000), target-now, deadline-now});
            const double dt = step / 1e6;
            std::array<double,4> distance{};
            for (uint8_t i=0;i<4;++i) {
                double desired = firmware_.pwm(i) * max_speed / 100;
                if (firmware_.reverse(i)) desired = -desired;
                const double tau = time_constant / (firmware_.mode(i)==1 ? 5 : 1);
                const double decay = std::exp(-dt/tau);
                distance[i] = desired*dt + (velocity[i]-desired)*tau*(1-decay);
                velocity[i] = desired+(velocity[i]-desired)*decay;
                angle[i] += distance[i];
            }
            for (uint8_t e=0;e<4;++e) {
                counts_[e] += distance[encoder_motors[e]] * firmware_.resolution(e) / 6.2831853071795864769;
                // Bound floating counter growth while retaining the fractional tick.
                counts_[e] = std::fmod(counts_[e],65536.);
                firmware_.encoder(e, static_cast<uint16_t>(static_cast<int32_t>(counts_[e])));
            }
            now += step;
            firmware_.shaft_velocity(velocity.data());
            poll();
        }
        poll();
    }
};
