#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

namespace wirehawk_bridge {

// Hard cap on the fixed-size arrays. The active count is Config::num_motors.
constexpr int MAX_MOTORS = 8;

// Bridge configuration, populated from ROS parameters (single source of truth
// in the yaml launch file — no hardcoded values here beyond safe defaults that
// only apply when a param is absent).
struct Config {
    std::string interface = "eth0";
    int64_t cycle_ns       = 1000000;    // 1 ms EtherCAT cycle
    int32_t counts_per_rev = 131072;     // 17-bit encoder
    int     num_motors     = 1;
    int     rt_cpu         = 3;          // core the RT thread is pinned to
    double  max_speed      = 131072.0;   // counts/s   (trapezoid speed limit)
    double  max_accel      = 1310720.0;  // counts/s^2 (trapezoid accel limit)
};

// Shared state between the RT thread (SOEM loop) and the ROS2 thread.
// "Latest value" pattern: both sides hold `mtx` only for a few-byte copy, and
// never during blocking calls. The std::atomic fields are lock-free and do not
// need the mutex.
struct BridgeData {
    // command: ROS2 -> RT
    std::array<int32_t, MAX_MOTORS> target_position{};
    std::atomic<bool> target_valid{false};   // a command has been received at least once

    // feedback: RT -> ROS2
    std::array<int32_t, MAX_MOTORS> actual_position{};
    std::array<int16_t, MAX_MOTORS> actual_torque{};
    std::array<uint16_t, MAX_MOTORS> status_word{};
    std::array<uint16_t, MAX_MOTORS> error_code{};

    // lifecycle / diagnostics
    std::atomic<bool> running{true};        // RT loop runs while true
    std::atomic<bool> enabled{false};       // drives reached "operation enabled"

    std::mutex mtx;                         // protects the arrays above (not the atomics)
};

// Runs the SOEM CSP loop in the calling thread (intended to be a dedicated RT
// thread). Blocks until BridgeData::running is cleared. Returns 0 on clean
// shutdown, non-zero on a fatal init error.
int soem_rt_loop(const Config& cfg, BridgeData* data);

} // namespace wirehawk_bridge
