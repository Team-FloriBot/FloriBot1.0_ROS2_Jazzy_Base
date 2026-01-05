#ifndef HARDWARE_NODE_HPP
#define HARDWARE_NODE_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "base/msg/wheel_velocities.hpp"
#include "base/diff_drive_lib.hpp"

class HardwareNode : public rclcpp::Node
{
public:
    HardwareNode();

private:
    // Callback für eingehende Wheel-Commands
    void command_callback(const base::msg::WheelVelocities::SharedPtr msg);

    // Hauptregel-Loop (Timer)
    void control_loop();

    // ROS
    rclcpp::Subscription<base::msg::WheelVelocities>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Hardware
    std::unique_ptr<SSC32Driver> motor_driver_;
    std::unique_ptr<PhidgetEncoderWrapper> enc_left_;
    std::unique_ptr<PhidgetEncoderWrapper> enc_right_;

    // PID-Regler
    std::unique_ptr<PIDController> pid_left_;
    std::unique_ptr<PIDController> pid_right_;

    // Zielgeschwindigkeiten
    double target_l_{0.0};
    double target_r_{0.0};

    // Letzte Positionen (für Geschwindigkeitsermittlung)
    // 1440 Ticks pro Motorumdrehung * 20 (Getriebeübersetzung) = 28800 Ticks pro Radumdrehung
    static constexpr double gear_ratio_ = 20.0;
    static constexpr double ticks_per_rev_ = 1440.0 * gear_ratio_; 
    static constexpr double ticks_to_rad_ = (2.0 * M_PI) / ticks_per_rev_;
    double last_pos_l_ = enc_left_->get_position() * ticks_to_rad_;
    double last_pos_r_ = enc_right_->get_position() * ticks_to_rad_;

    // Zeitstempel
    rclcpp::Time last_time_;
    rclcpp::Time last_cmd_time_;  // <--- hinzugefügt

    // Sicherheitsparameter
    double max_wheel_speed_{1.0}; // <--- hinzugefügt, kann über Parameter gesetzt werden

    // Thread-Sicherheit
    std::mutex mtx_;
};

#endif  // HARDWARE_NODE_HPP
