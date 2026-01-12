#include "base/diff_drive_lib.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <stdexcept>
#include <iostream>
#include <cerrno>
#include <cstring>
#include <algorithm>
#include <cmath>

// ======================================================
// PID Controller
// ======================================================

PIDController::PIDController(
    double kp,
    double ki,
    double kd,
    double output_limit,
    double deadband
)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  out_lim_(std::abs(output_limit)),
  deadband_(std::abs(deadband))
{
}

void PIDController::reset()
{
    integrator_ = 0.0;
    last_output_ = 0.0;
    prev_error_ = 0.0;
    first_run_ = true;
}

double PIDController::compute(double target_setpoint, double measured, double dt)
{
    if (dt <= 0.0) {
        return last_output_;
    }

    // --- RAMPE BERECHNEN ---
    // Wie viel darf sich der Sollwert in diesem Zeitschritt maximal ändern?
    double max_change = max_accel_ * dt;
    double error_setpoint = target_setpoint - ramped_setpoint_;

    if (first_run_){
        ramped_setpoint_ = measured;
    }

    // Begrenze die Änderung des ramped_setpoint_ auf max_change
    if (error_setpoint > max_change) {
        ramped_setpoint_ += max_change;
    } else if (error_setpoint < -max_change) {
        ramped_setpoint_ -= max_change;
    } else {
        ramped_setpoint_ = target_setpoint;
    }

    const double error = ramped_setpoint_ - measured;

    // P-Anteil
    double P_term = kp_ * error;

    // I-Anteil
    integrator_ += ki_ * error * dt;

    // D-Anteil
    double derivative = 0.0;
    if (!first_run_) {
        derivative = (error - prev_error_) / dt;
    }
    double D_term = kd_ * derivative; 


    // PID-Rohwert
    double output = P_term + integrator_ + D_term;

    // Sättigung + Anti-Windup (Integrator-Clamping)
    if (output > out_lim_) {
        output = out_lim_;
        integrator_ -= ki_ * error * dt;
    }
    else if (output < -out_lim_) {
        output = -out_lim_;
        integrator_ -= ki_ * error * dt;
    }

    // Zustand aktualisieren
    prev_error_ = error;
    last_output_ = output;
    first_run_ = false;

    return output;
}


// ======================================================
// SSC-32 Driver
// ======================================================

SSC32Driver::SSC32Driver(const std::string& port, int baudrate)
: fd_(-1)
{
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        throw std::runtime_error(
            "SSC32: Failed to open serial port " + port + ": " +
            std::strerror(errno)
        );
    }

    struct termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
        close(fd_);
        throw std::runtime_error("SSC32: tcgetattr failed");
    }

    // Baudrate (aktuell nur 115200 unterstützt)
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;        // 8 Datenbits
    tty.c_cflag &= ~PARENB;    // keine Parität
    tty.c_cflag &= ~CSTOPB;    // 1 Stopbit
    tty.c_cflag &= ~CRTSCTS;   // kein HW-Flowcontrol

    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5; // 0.5s Timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close(fd_);
        throw std::runtime_error("SSC32: tcsetattr failed");
    }
}

SSC32Driver::~SSC32Driver()
{
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

void SSC32Driver::send_commands(int pwm_left, int pwm_right)
{
    if (fd_ < 0) return;

    // Begrenzen (Sicherheitsnetz)
    if (pwm_left < 1000)  pwm_left = 1000;
    if (pwm_left > 2000)  pwm_left = 2000;

    if (pwm_right < 1000) pwm_right = 1000;
    if (pwm_right > 2000) pwm_right = 2000;


    // T10 => Bewegung in 10ms (passend zu 100Hz Loop)
    const std::string cmd =
        "#0P" + std::to_string(pwm_left) +
        "#1P" + std::to_string(pwm_right) +
        "T10\r";

    const ssize_t written = write(fd_, cmd.c_str(), cmd.size());
    if (written < 0) {
        std::cerr << "[SSC32] Serial write error: "
                  << std::strerror(errno) << std::endl;
    }
}

// ======================================================
// Phidget Encoder
// ======================================================

// Attach Callback (Library-intern)
int CCONV AttachHandler(CPhidgetHandle phid, void*)
{
    int serial = -1;
    CPhidget_getSerialNumber(phid, &serial);
    std::cout << "[Phidget] Encoder attached (SN=" << serial << ")" << std::endl;
    return 0;
}

PhidgetEncoderWrapper::PhidgetEncoderWrapper(int expected_serial)
: handle_(nullptr)
{
    CPhidgetEncoderHandle h = nullptr;
    int result = 0;

    // Create
    result = CPhidgetEncoder_create(&h);
    if (result != 0 || !h) {
        throw std::runtime_error("Phidget: Encoder_create failed");
    }

    // Attach callback (wie ROS1)
    CPhidget_set_OnAttach_Handler(
        (CPhidgetHandle)h,
        AttachHandler,
        nullptr
    );

    // WICHTIG: exakt wie im ROS1-Code
    result = CPhidget_open((CPhidgetHandle)h, -1);
    if (result != 0) {
        CPhidget_delete((CPhidgetHandle)h);
        throw std::runtime_error("Phidget: open(-1) failed");
    }

    // Warten auf Hardware
    result = CPhidget_waitForAttachment((CPhidgetHandle)h, 10000);
    if (result != 0) {
        const char* err = nullptr;
        CPhidget_getErrorDescription(result, &err);
        CPhidget_close((CPhidgetHandle)h);
        CPhidget_delete((CPhidgetHandle)h);
        throw std::runtime_error(
            "Phidget attach timeout: " +
            std::string(err ? err : "unknown error")
        );
    }

    // Seriennummer prüfen
    int actual_serial = -1;
    CPhidget_getSerialNumber((CPhidgetHandle)h, &actual_serial);

    if (actual_serial != expected_serial) {
        CPhidget_close((CPhidgetHandle)h);
        CPhidget_delete((CPhidgetHandle)h);
        throw std::runtime_error(
            "Phidget wrong serial. Expected " +
            std::to_string(expected_serial) +
            ", got " +
            std::to_string(actual_serial)
        );
    }

    // Initialer Read (Sanity)
    int pos = 0;
    CPhidgetEncoder_getPosition(h, 0, &pos);
    last_known_pos_ = pos;

    handle_ = h;
}

PhidgetEncoderWrapper::~PhidgetEncoderWrapper()
{
    if (handle_) {
        CPhidget_close((CPhidgetHandle)handle_);
        CPhidget_delete((CPhidgetHandle)handle_);
        handle_ = nullptr;
    }
}

int PhidgetEncoderWrapper::get_position()
{
    if (!handle_) return last_known_pos_;

    int pos = 0;
    const int r = CPhidgetEncoder_getPosition(handle_, 0, &pos);
    if (r != 0) {
        return last_known_pos_;
    }

    last_known_pos_ = pos;
    return pos;
}

