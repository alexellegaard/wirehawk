// ethercat_bridge_node.cpp — the ROS2 side of the bridge.
// Spawns the RT thread (soem_rt_loop), then bridges ROS2 topics <-> BridgeData.
// The RT thread never touches ROS2; this node never touches EtherCAT timing.

#include "wirehawk_ethercat_bridge/bridge.hpp"
#include "wirehawk_msgs/msg/motor_command.hpp"
#include "wirehawk_msgs/msg/motor_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <thread>

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
    node->declare_parameter("counts_offset", cfg.counts_offset);

    cfg.interface      = node->get_parameter("interface").as_string();
    cfg.cycle_ns       = node->get_parameter("cycle_ns").as_int();
    cfg.counts_per_rev = node->get_parameter("counts_per_rev").as_int();
    cfg.num_motors     = node->get_parameter("num_motors").as_int();
    cfg.rt_cpu         = node->get_parameter("rt_cpu").as_int();
    cfg.max_speed      = node->get_parameter("max_speed").as_double();
    cfg.max_accel      = node->get_parameter("max_accel").as_double();
    cfg.counts_offset  = node->get_parameter("counts_offset").as_int();

    BridgeData data;

    // ---- spawn the RT thread ----
    std::thread rt(wirehawk_bridge::soem_rt_loop, cfg, &data);

    // ---- publishers / subscribers ----
    auto state_pub = node->create_publisher<wirehawk_msgs::msg::MotorState>("state/motors", 10);

    auto cmd_sub = node->create_subscription<wirehawk_msgs::msg::MotorCommand>(
        "cmd/motors", 10,
        [&data, &cfg](wirehawk_msgs::msg::MotorCommand::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(data.mtx);
            int n = std::min<int>(msg->position.size(), cfg.num_motors);
            for (int i = 0; i < n; i++)
                data.target_position[i] =
                    (int32_t)((int64_t)msg->position[i] + cfg.counts_offset);
            data.target_valid = true;
        });

    // state publisher at ~100 Hz
    auto timer = node->create_wall_timer(std::chrono::milliseconds(10), [&]() {
        wirehawk_msgs::msg::MotorState msg;
        {
            std::lock_guard<std::mutex> lk(data.mtx);
            for (int i = 0; i < cfg.num_motors; i++) {
                msg.position.push_back(data.actual_position[i]);
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
