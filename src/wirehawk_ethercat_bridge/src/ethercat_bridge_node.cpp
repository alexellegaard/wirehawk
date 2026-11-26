// ethercat_bridge_node.cpp — the ROS2 side of the bridge.
// Spawns the RT thread (soem_rt_loop), then bridges ROS2 topics <-> BridgeData.
// The RT thread never touches ROS2; this node never touches EtherCAT timing.

#include "wirehawk_ethercat_bridge/bridge.hpp"
#include "wirehawk_msgs/msg/motor_command.hpp"
#include "wirehawk_msgs/msg/motor_state.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

using wirehawk_bridge::BridgeData;
using wirehawk_bridge::Config;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ethercat_bridge");

    // ---- parameters (single source of truth: config yaml, no hardcoded values) ----
    Config cfg;
    node->declare_parameter("interface",     cfg.interface);
    node->declare_parameter("cycle_ns",      cfg.cycle_ns);
    node->declare_parameter("counts_per_rev", cfg.counts_per_rev);
    node->declare_parameter("num_motors",    cfg.num_motors);
    node->declare_parameter("rt_cpu",        cfg.rt_cpu);
    node->declare_parameter("max_speed",     cfg.max_speed);
    node->declare_parameter("max_accel",     cfg.max_accel);
    node->declare_parameter("pos_feedback_gain", cfg.pos_feedback_gain);
    node->declare_parameter("gear_ratio",    cfg.gear_ratio);
    node->declare_parameter("mast_run",      cfg.mast_run);
    node->declare_parameter("spool_total_m", cfg.spool.total_m);
    node->declare_parameter("spool_core_d_m", cfg.spool.core_d_m);
    node->declare_parameter("spool_rope_d_m", cfg.spool.rope_d_m);
    node->declare_parameter("spool_turns_per_layer", cfg.spool.turns_per_layer);
    node->declare_parameter("spool_max_layers", cfg.spool.max_layers);
    node->declare_parameter("zero_ticks", std::vector<int64_t>{0, 0, 0, 0, 0, 0, 0, 0});
    node->declare_parameter("direction", std::vector<int64_t>{1, 1, 1, 1, 1, 1, 1, 1});
    node->declare_parameter("max_step", cfg.max_step);
    node->declare_parameter("max_following_error", cfg.max_following_error);
    node->declare_parameter("max_torque", cfg.max_torque);

    cfg.interface      = node->get_parameter("interface").as_string();
    cfg.cycle_ns       = node->get_parameter("cycle_ns").as_int();
    cfg.counts_per_rev = node->get_parameter("counts_per_rev").as_int();
    cfg.num_motors     = node->get_parameter("num_motors").as_int();
    cfg.rt_cpu         = node->get_parameter("rt_cpu").as_int();
    cfg.max_speed      = node->get_parameter("max_speed").as_double();
    cfg.max_accel      = node->get_parameter("max_accel").as_double();
    cfg.pos_feedback_gain = node->get_parameter("pos_feedback_gain").as_double();
    cfg.gear_ratio     = node->get_parameter("gear_ratio").as_double();
    cfg.mast_run       = node->get_parameter("mast_run").as_double();
    cfg.spool.total_m  = node->get_parameter("spool_total_m").as_double();
    cfg.spool.core_d_m = node->get_parameter("spool_core_d_m").as_double();
    cfg.spool.rope_d_m = node->get_parameter("spool_rope_d_m").as_double();
    cfg.spool.turns_per_layer = node->get_parameter("spool_turns_per_layer").as_int();
    cfg.spool.max_layers = node->get_parameter("spool_max_layers").as_int();
    {
        auto z = node->get_parameter("zero_ticks").as_integer_array();
        for (size_t i = 0; i < cfg.zero_ticks.size(); i++)
            cfg.zero_ticks[i] = (i < z.size()) ? (int32_t)z[i] : 0;
    }
    {
        auto d = node->get_parameter("direction").as_integer_array();
        for (size_t i = 0; i < cfg.direction.size(); i++)
            cfg.direction[i] = (i < d.size() && d[i] < 0) ? -1 : 1;
    }
    cfg.max_step = node->get_parameter("max_step").as_int();
    cfg.max_following_error = node->get_parameter("max_following_error").as_int();
    cfg.max_torque = node->get_parameter("max_torque").as_int();

    BridgeData data;

    // ---- spawn the RT thread ----
    std::thread rt(wirehawk_bridge::soem_rt_loop, cfg, &data);

    // ---- publishers / subscribers ----
    auto state_pub = node->create_publisher<wirehawk_msgs::msg::MotorState>("state/motors", 10);

    // Schema-version heartbeat on a STABLE std_msgs type: readable even when the
    // MotorState schema is mismatched (the controller checks it and refuses to
    // command if it disagrees — see bridge.hpp::kSchemaVersion).
    auto schema_pub = node->create_publisher<std_msgs::msg::UInt32>("/wirehawk/schema_version", 10);
    auto schema_timer = node->create_wall_timer(std::chrono::seconds(1), [&]() {
        std_msgs::msg::UInt32 v;
        v.data = wirehawk_bridge::kSchemaVersion;
        schema_pub->publish(v);
    });

    auto cmd_sub = node->create_subscription<wirehawk_msgs::msg::MotorCommand>(
        "cmd/motors", 10,
        [&data, &cfg](wirehawk_msgs::msg::MotorCommand::SharedPtr msg) {
            // Feedforward velocity: d(position)/dt at the command rate, so the
            // RT interpolator can track the command's MOTION instead of deriving
            // it from the position gap (which caused a 100 Hz sawtooth).
            static auto last_t = std::chrono::steady_clock::now();
            static std::array<int32_t, wirehawk_bridge::MAX_MOTORS> last_pos{};
            static bool has_last = false;
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_t).count();

            std::lock_guard<std::mutex> lk(data.mtx);
            int n = std::min<int>(msg->position.size(), cfg.num_motors);
            for (int i = 0; i < n; i++) {
                // free span [m] -> raw 0x607A target (spool model + zero + mast_run)
                int32_t p = wirehawk_bridge::free_span_to_raw((double)msg->position[i], i, cfg);
                if (has_last && dt > 1e-6)
                    data.target_velocity[i] = (double)(p - last_pos[i]) / dt;
                last_pos[i] = p;
                data.target_position[i] = p;
            }
            has_last = true;
            last_t = now;
            data.target_valid = true;
        });

    // state publisher at ~100 Hz
    auto timer = node->create_wall_timer(std::chrono::milliseconds(10), [&]() {
        wirehawk_msgs::msg::MotorState msg;
        {
            std::lock_guard<std::mutex> lk(data.mtx);
            for (int i = 0; i < cfg.num_motors; i++) {
                // raw 0x6064 -> free span [m] (spool model + zero + mast_run)
                msg.position.push_back((float)wirehawk_bridge::raw_to_free_span(data.actual_position[i], i, cfg));
                msg.target.push_back((float)wirehawk_bridge::raw_to_free_span(data.commanded_target[i], i, cfg));
                msg.torque.push_back(data.actual_torque[i]);
                msg.status_word.push_back(data.status_word[i]);
                msg.error_code.push_back(data.error_code[i]);
            }
        }
        state_pub->publish(msg);
    });

    RCLCPP_INFO(node->get_logger(),
                "ethercat_bridge up: iface=%s, %d motor(s), %ld Hz RT loop on core %d",
                cfg.interface.c_str(), cfg.num_motors,
                (long)(1000000000L / cfg.cycle_ns), cfg.rt_cpu);

    rclcpp::spin(node);

    // ---- clean shutdown ----
    data.running = false;
    if (rt.joinable())
        rt.join();
    rclcpp::shutdown();
    return 0;
}
