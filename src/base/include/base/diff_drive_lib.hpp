#ifndef DIFF_DRIVE_LIB_HPP
#define DIFF_DRIVE_LIB_HPP

#include <string>
#include <phidget21.h>
#include <mutex>

class PIDController {
public:
    PIDController(
        double kp,
        double ki,
        double kd,
        double output_limit = 1.0,
        double deadband = 0.05
    );

    /// Reset interner Zustände (Integrator, Historie)
    void reset();

    /// Berechnet normierte Stellgröße [-output_limit .. +output_limit]
    double compute(double setpoint, double measured, double dt);


private:
    // Parameter
    double kp_;
    double ki_;
    double kd_;
    double out_lim_;
    double deadband_;

    // Zustände
    double integrator_ = 0.0;
    double prev_error_ = 0.0;
    double last_output_ = 0.0;
    double max_accel_ = 2.0;
    double ramped_setpoint_ = 0.0;
    bool first_run_ = true;
};

class SSC32Driver {
public:
    SSC32Driver(const std::string& port, int baudrate);
    ~SSC32Driver();
    void send_commands(int pwm_left, int pwm_right);
private:
    int fd_;
};

class PhidgetEncoderWrapper {
public:
    PhidgetEncoderWrapper(int serial_number);
    ~PhidgetEncoderWrapper();
    int get_position();
private:
    CPhidgetEncoderHandle handle_;
    int last_known_pos_ = 0;
};

#endif
