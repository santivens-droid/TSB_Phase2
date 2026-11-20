#ifndef TELEOP_NODE_HPP_
#define TELEOP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tsb_recruitment_interfaces/msg/agent_command.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <cmath>

// Topic name defines
#define AGENT_COMMAND_TOPIC "command"

// Frequency defines (in Hz)
#define TELEOP_PUBLISH_FREQUENCY 10.0

// Control mode defines
#define CONTROL_MODE_RPM "rpm"
#define CONTROL_MODE_SURGE_YAW "surge_yaw"

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode();
    ~TeleopNode();

private:
    void initializeParameters();
    void initializePublishers();
    void initializeTimer();
    void setup_terminal();
    void restore_terminal();
    void timer_callback();
    void process_key(char c);

    rclcpp::Publisher<tsb_recruitment_interfaces::msg::AgentCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios old_tio_;

    // Control mode
    std::string control_mode_;

    // RPM control variables
    double left_rpm_ = 0.0;
    double right_rpm_ = 0.0;

    // Surge/Yaw control variables
    double surge_ = 0.0;
    double yaw_ = 0.0;

    // Parameters
    double rpm_increment_;    // RPM increment per key press
    double surge_increment_;  // Surge increment per key press (m/s)
    double yaw_increment_; // Yaw increment per key press (degrees)
    double max_rpm_;          // Max RPM (only used in RPM mode)
    double max_surge_;        // Max surge (only used in surge_yaw mode)
};

#endif  // TELEOP_NODE_HPP_