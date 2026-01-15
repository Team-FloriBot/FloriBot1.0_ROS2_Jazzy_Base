#include "base/hardware_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

HardwareNode::HardwareNode()
: Node("hardware_node")
{
    // --------------------------------------------------
    // Parameter deklarieren
    // --------------------------------------------------
    this->declare_parameter("serial_port", "/dev/ttyS1");
    this->declare_parameter("left_enc_serial", 101902);
    this->declare_parameter("right_enc_serial", 102191);

    this->declare_parameter("kp_r", 0.02);
    this->declare_parameter("ki_r", 0.0);
    this->declare_parameter("kd_r", 0.0);

    this->declare_parameter("kp_l", 0.02);
    this->declare_parameter("ki_l", 0.0);
    this->declare_parameter("kd_l", 0.0);

    this->declare_parameter("max_wheel_speed", 10.0); // [rad/s]

    const std::string serial_port = this->get_parameter("serial_port").as_string();
    const int left_sn  = this->get_parameter("left_enc_serial").as_int();
    const int right_sn = this->get_parameter("right_enc_serial").as_int();

    max_wheel_speed_ = this->get_parameter("max_wheel_speed").as_double();

    // --------------------------------------------------
    // Hardware initialisieren
    // --------------------------------------------------
    try {
        motor_driver_ = std::make_unique<SSC32Driver>(serial_port, 115200);
        enc_left_     = std::make_unique<PhidgetEncoderWrapper>(left_sn);
        enc_right_    = std::make_unique<PhidgetEncoderWrapper>(right_sn);

        last_pos_l_ = enc_left_->get_position() * ticks_to_rad_;
        last_pos_r_ = enc_right_->get_position() * ticks_to_rad_;

        RCLCPP_INFO(
            this->get_logger(),
            "Hardware initialisiert (SSC32 + Encoder %d / %d)",
            left_sn, right_sn
        );
    } catch (const std::exception& e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Hardware-Initialisierung fehlgeschlagen: %s",
            e.what()
        );
        throw;
    }

    // --------------------------------------------------
    // PID Controller initialisieren
    // --------------------------------------------------
    double kp_r = this->get_parameter("kp_r").as_double();
    double ki_r = this->get_parameter("ki_r").as_double();
    double kd_r = this->get_parameter("kd_r").as_double();

    double kp_l = this->get_parameter("kp_l").as_double();
    double ki_l = this->get_parameter("ki_l").as_double();
    double kd_l = this->get_parameter("kd_l").as_double();

    pid_left_  = std::make_unique<PIDController>(kp_l, ki_l, kd_l);
    pid_right_ = std::make_unique<PIDController>(kp_r, ki_r, kd_r);

    // --------------------------------------------------
    // Subscriber / Publisher
    // --------------------------------------------------
    sub_ = this->create_subscription<base::msg::WheelVelocities>(
        "/wheel_commands",
        10,
        std::bind(&HardwareNode::command_callback, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/wheel_states",
        10
    );

    last_time_ = this->now();
    last_cmd_time_ = last_time_;

    // --------------------------------------------------
    // Timer für Regelung
    // --------------------------------------------------
    timer_ = this->create_wall_timer(
        10ms,
        std::bind(&HardwareNode::control_loop, this)
    );
}

void HardwareNode::command_callback(
    const base::msg::WheelVelocities::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    target_l_ = msg->left;
    target_r_ = msg->right;
    last_cmd_time_ = this->now();
}

void HardwareNode::control_loop()
{
    if (!motor_driver_ || !enc_left_ || !enc_right_) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Hardware nicht bereit – control_loop übersprungen"
        );
        return;
    }

    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) return;
    last_time_ = now;

    // --------------------------------------------------
    // Encoder in Rad umrechnen
    // --------------------------------------------------

    double curr_pos_l = enc_left_->get_position() * ticks_to_rad_;
    double curr_pos_r = enc_right_->get_position() * ticks_to_rad_;


    double vel_l = (curr_pos_l - last_pos_l_) / dt;
    double vel_r = (curr_pos_r - last_pos_r_) / dt;

    //Schutz gegen Encoder Spikes
    vel_l = std::clamp(vel_l, -3.0 * max_wheel_speed_, 3.0 * max_wheel_speed_);
    vel_r = std::clamp(vel_r, -3.0 * max_wheel_speed_, 3.0 * max_wheel_speed_);


    //lowpassfilter for encoder signals
    constexpr double tau = 0.01; // 10 ms
    vel_l_filt_ = lowpass(vel_l, vel_l_filt_, dt, tau);
    vel_r_filt_ = lowpass(vel_r, vel_r_filt_, dt, tau);

    // --------------------------------------------------
    // Zielwerte aus Mutex kopieren
    // --------------------------------------------------
    double target_l, target_r;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        target_l = target_l_;
        target_r = target_r_;
    }

    // Variablen für den normierten Output (-1.0 bis 1.0)
    double out_l = 0.0;
    double out_r = 0.0;

    // --------------------------------------------------
    // Watchdog / PID Reset
    // --------------------------------------------------
    if ((now - last_cmd_time_).seconds() > 0.3) {
        pid_left_->reset();
        pid_right_->reset();
        target_l = 0.0;
        target_r = 0.0;
        vel_l_filt_ = 0.0;
        vel_r_filt_ = 0.0;
    }

    // --------------------------------------------------
    // Normierung auf [-1, 1] für PID
    // --------------------------------------------------
    auto clamp = [](double v) { return std::max(-1.0, std::min(1.0, v)); };

    if (open_loop_) {
        // --- OPEN-LOOP MODUS ---
        // Direkte Skalierung ohne PID-Einfluss
        out_l = std::clamp(target_l / max_wheel_speed_, -1.0, 1.0);
        out_r = std::clamp(target_r / max_wheel_speed_, -1.0, 1.0);
    }

    else{

        double target_l_norm = clamp(target_l / max_wheel_speed_);
        double target_r_norm = clamp(target_r / max_wheel_speed_);
        double vel_l_norm    = clamp(vel_l_filt_ / max_wheel_speed_);
        double vel_r_norm    = clamp(vel_r_filt_ / max_wheel_speed_);
    

        // --------------------------------------------------
        // PID Berechnung
        // --------------------------------------------------
        double out_l = pid_left_->compute(target_l_norm, vel_l_norm, dt);
        double out_r = pid_right_->compute(target_r_norm, vel_r_norm, dt);

        // Deadband
        if (std::abs(out_l) < 0.05) out_l = 0.0;
        if (std::abs(out_r) < 0.05) out_r = 0.0;

    }    

    int pwm_left  = 1500 - static_cast<int>(out_l * 500.0);
    int pwm_right = 1500 + static_cast<int>(out_r * 500.0);

    

    motor_driver_->send_commands(pwm_left, pwm_right);

    // --------------------------------------------------
    // JointState publizieren
    // --------------------------------------------------
    sensor_msgs::msg::JointState state;
    state.header.stamp = now;
    state.name = {"left_wheel", "right_wheel"};
    state.position = {curr_pos_l, curr_pos_r};
    state.velocity = {vel_l_filt_, vel_r_filt_};

    pub_->publish(state);

    // --------------------------------------------------
    // letzte Position speichern
    // --------------------------------------------------
    last_pos_l_ = curr_pos_l;
    last_pos_r_ = curr_pos_r;
}

double HardwareNode::lowpass(double v_raw, double v_prev, double dt, double tau)
{
    if (dt <= 0.0) return v_prev;
    const double alpha = dt / (tau + dt);
    return alpha * v_raw + (1.0 - alpha) * v_prev;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareNode>());
    rclcpp::shutdown();
    return 0;
}