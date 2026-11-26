#pragma once
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

namespace wirehawk_bridge {

// Hard cap on the fixed-size arrays. The active count is Config::num_motors.
constexpr int MAX_MOTORS = 8;

// Schema version of wirehawk_msgs/msg/MotorState. Bump this AND
// cdpr_node.py::SCHEMA_VERSION together whenever MotorState.msg changes. The
// bridge publishes it on /wirehawk/schema_version (a std_msgs type, so it stays
// readable even when MotorState itself is mismatched) and the controller refuses
// to command motion on a mismatch — a stale Pi's MotorState otherwise
// deserializes to garbage and the controller runs blind (field spool break).
constexpr uint32_t kSchemaVersion = 3;

// Drum geometry — the layer model mapping encoder counts <-> cable length.
// Ported from wirehawk_spool/spool_model.py; MUST stay in sync with spool.yaml.
struct SpoolSpec {
    double total_m = 65.0;          // cable inventory per winch [m]
    double core_d_m = 0.020;        // drum core diameter [m]
    double rope_d_m = 0.002;        // cable diameter [m]
    int    turns_per_layer = 70;    // turns per layer (ideal square packing)
    int    max_layers = 9;          // max winding layers
};

namespace {
constexpr double kPi = 3.14159265358979323846;

inline double spool_layer_circumference(const SpoolSpec& s, int layer) {
    return kPi * (s.core_d_m + (2.0 * layer - 1.0) * s.rope_d_m);
}

inline double spool_wound_to_turns(double wound, const SpoolSpec& s) {
    double remaining = wound, turns = 0.0;
    for (int layer = 1; layer <= s.max_layers; layer++) {
        if (remaining <= 1e-12) break;
        double circ = spool_layer_circumference(s, layer);
        double len = std::min(remaining, (double)s.turns_per_layer * circ);
        turns += len / circ;
        remaining -= len;
    }
    return turns;
}

inline double spool_turns_to_wound(double turns, const SpoolSpec& s) {
    double remaining = turns, wound = 0.0;
    for (int layer = 1; layer <= s.max_layers; layer++) {
        double circ = spool_layer_circumference(s, layer);
        if (remaining <= (double)s.turns_per_layer + 1e-12) {
            wound += std::max(0.0, remaining) * circ;
            remaining = 0.0;
            break;
        }
        wound += (double)s.turns_per_layer * circ;
        remaining -= (double)s.turns_per_layer;
    }
    return wound;
}

inline double spool_length_to_counts(double length_m, const SpoolSpec& s,
                                     int32_t counts_per_rev, double gear_ratio) {
    return std::round(spool_wound_to_turns(s.total_m - length_m, s)
                      * (double)counts_per_rev * gear_ratio);
}

inline double spool_counts_to_length(double counts, const SpoolSpec& s,
                                     int32_t counts_per_rev, double gear_ratio) {
    return s.total_m - spool_turns_to_wound(counts / ((double)counts_per_rev * gear_ratio), s);
}
}  // namespace

// Bridge configuration, populated from ROS parameters (single source of truth
// in the yaml launch file — no hardcoded values here beyond safe defaults that
// only apply when a param is absent).
struct Config {
    std::string interface = "eth0";
    int64_t cycle_ns       = 1000000;    // 1 ms EtherCAT cycle
    int32_t counts_per_rev = 131072;     // 17-bit encoder
    int     num_motors     = 1;
    int     rt_cpu         = 3;
    double  max_speed      = 131072.0;   // counts/s   (trapezoid speed limit)
    double  max_accel      = 1310720.0;  // counts/s^2 (trapezoid accel limit)
    double  pos_feedback_gain = 20.0;    // 1/s: position drift correction on top of velocity feedforward
    double  gear_ratio     = 1.0;        // motor revs per drum rev
    double  mast_run       = 3.0;        // winch-drum -> top-pulley cable [m]
    SpoolSpec spool;                     // drum geometry (counts <-> length)
    std::array<int32_t, MAX_MOTORS> zero_ticks{};  // raw 0x6064 at "rope end at the pulley" (slave order)
    std::array<int, MAX_MOTORS> direction{};       // winding sign: +1 normal, -1 reversed
    int64_t max_step = 0;                         // counts: max travel a single command may demand (0 = disabled)
    int64_t max_following_error = 1000000;        // counts (~0.48 m): watchdog — disable all motors on violation
    int16_t max_torque = 0;                       // 0.1%-rated torque units: watchdog — disable all motors on |0x6077| exceed (0 = disabled)
    int16_t pre_tension_torque = 0;               // 0.1%-rated: absolute |0x6077| that marks a cable taut (0 = skip pre-tension)
    int32_t pre_tension_step = 0;                 // raw counts/cycle wound in during pre-tension (0 = skip pre-tension)
};

// raw 0x6064 -> free-span cable length [m] (anchor/pulley -> TCP). The bridge
// owns this conversion (zero + mast_run + spool model + direction), so the
// controller works purely in free-span metres and never sees encoder counts.
inline double raw_to_free_span(int32_t raw, int idx, const Config& cfg) {
    double delta = (double)((int64_t)raw - (int64_t)cfg.zero_ticks[idx]) * (double)cfg.direction[idx];
    double total_counts = delta + spool_length_to_counts(cfg.mast_run, cfg.spool, cfg.counts_per_rev, cfg.gear_ratio);
    return spool_counts_to_length(total_counts, cfg.spool, cfg.counts_per_rev, cfg.gear_ratio) - cfg.mast_run;
}

// free-span cable length [m] -> raw 0x607A target. Exact inverse of raw_to_free_span.
inline int32_t free_span_to_raw(double free_span, int idx, const Config& cfg) {
    double total = spool_length_to_counts(cfg.mast_run + free_span, cfg.spool, cfg.counts_per_rev, cfg.gear_ratio);
    double mast  = spool_length_to_counts(cfg.mast_run, cfg.spool, cfg.counts_per_rev, cfg.gear_ratio);
    int64_t raw = (int64_t)cfg.zero_ticks[idx] + (int64_t)cfg.direction[idx] * (int64_t)llround(total - mast);
    return (int32_t)raw;
}

// Shared state between the RT thread (SOEM loop) and the ROS2 thread.
// "Latest value" pattern: both sides hold `mtx` only for a few-byte copy, and
// never during blocking calls. The std::atomic fields are lock-free and do not
// need the mutex.
//
// Everything here is in RAW encoder counts: the RT thread reads/writes the drive
// directly, and the ROS2 node does the raw <-> free-span conversion at the wire.
struct BridgeData {
    // command: ROS2 -> RT
    std::array<int32_t, MAX_MOTORS> target_position{};   // raw 0x607A target (ROS node converted free span -> raw)
    std::array<double, MAX_MOTORS> target_velocity{};    // counts/s, computed at cmd rate (feedforward)
    std::atomic<bool> target_valid{false};               // a command has been received at least once

    // feedback: RT -> ROS2
    std::array<int32_t, MAX_MOTORS> actual_position{};   // raw 0x6064 (ROS node converts to free span)
    std::array<int32_t, MAX_MOTORS> commanded_target{};  // interpolated target actually written to 0x607A
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
