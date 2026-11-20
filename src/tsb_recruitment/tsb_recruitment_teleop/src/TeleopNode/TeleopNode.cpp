#include "tsb_recruitment_teleop/TeleopNode.hpp"

TeleopNode::TeleopNode() : Node("teleop_node")
{
    initializeParameters();
    initializePublishers();
    initializeTimer();

    // Set terminal to raw mode
    setup_terminal();

    RCLCPP_INFO(this->get_logger(), "=== Teleop keyboard node started ===");

    if (control_mode_ == CONTROL_MODE_RPM) {
        RCLCPP_INFO(this->get_logger(), "Control Mode: RPM");
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  w / s   : forward / back");
        RCLCPP_INFO(this->get_logger(), "  a / d   : turn left / turn right");
        RCLCPP_INFO(this->get_logger(), "  space   : stop");
        RCLCPP_INFO(this->get_logger(), "  q       : quit");
    } else if (control_mode_ == CONTROL_MODE_SURGE_YAW) {
        RCLCPP_INFO(this->get_logger(), "Control Mode: Surge/Yaw");
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  w / s   : surge + / -");
        RCLCPP_INFO(this->get_logger(), "  a / d   : yaw + / -");
        RCLCPP_INFO(this->get_logger(), "  space   : stop");
        RCLCPP_INFO(this->get_logger(), "  q       : quit");
    }
}

TeleopNode::~TeleopNode()
{
    restore_terminal();
}

void TeleopNode::initializeParameters()
{
    // Declare parameters
    this->declare_parameter<std::string>("control_mode", CONTROL_MODE_SURGE_YAW);
    this->declare_parameter<double>("max_rpm", 500.0);
    this->declare_parameter<double>("rpm_increment", 10.0);
    this->declare_parameter<double>("max_surge", 2.0);
    this->declare_parameter<double>("surge_increment", 0.1);
    this->declare_parameter<double>("yaw_increment", 5.0);

    // Get parameters
    control_mode_ = this->get_parameter("control_mode").as_string();
    max_rpm_ = this->get_parameter("max_rpm").as_double();
    rpm_increment_ = this->get_parameter("rpm_increment").as_double();
    max_surge_ = this->get_parameter("max_surge").as_double();
    surge_increment_ = this->get_parameter("surge_increment").as_double();
    yaw_increment_ = this->get_parameter("yaw_increment").as_double();

    // Log parameters
    RCLCPP_INFO(this->get_logger(), "Control mode: %s", control_mode_.c_str());
    if (control_mode_ == CONTROL_MODE_RPM) {
        RCLCPP_INFO(this->get_logger(), "Parameters loaded: max_rpm=%.1f, rpm_increment=%.1f",
                    max_rpm_, rpm_increment_);
    } else if (control_mode_ == CONTROL_MODE_SURGE_YAW) {
        RCLCPP_INFO(this->get_logger(), "Parameters loaded: max_surge=%.1f, surge_increment=%.2f, yaw_increment=%.1f",
                    max_surge_, surge_increment_, yaw_increment_);
    }
}

void TeleopNode::initializePublishers()
{
    publisher_ = this->create_publisher<tsb_recruitment_interfaces::msg::AgentCommand>(
        AGENT_COMMAND_TOPIC, 10);
}

void TeleopNode::initializeTimer()
{
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / TELEOP_PUBLISH_FREQUENCY)),
        std::bind(&TeleopNode::timer_callback, this));
}

void TeleopNode::setup_terminal()
{
    // Get current terminal settings
    tcgetattr(STDIN_FILENO, &old_tio_);

    // Set new terminal settings
    struct termios new_tio = old_tio_;
    new_tio.c_lflag &= (~ICANON & ~ECHO);  // Disable canonical mode and echo
    new_tio.c_cc[VMIN] = 0;                // Read at least 0 characters
    new_tio.c_cc[VTIME] = 0;               // No timeout

    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    // Set stdin to non-blocking
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void TeleopNode::restore_terminal()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
}

void TeleopNode::timer_callback()
{
    char c;
    if (read(STDIN_FILENO, &c, 1) > 0) {
        process_key(c);
    }

    // Publish current command
    auto msg = tsb_recruitment_interfaces::msg::AgentCommand();
    msg.header.stamp = this->now();

    if (control_mode_ == CONTROL_MODE_RPM) {
        msg.command_values = {left_rpm_, right_rpm_};
    } else if (control_mode_ == CONTROL_MODE_SURGE_YAW) {
        // Convert yaw from degrees to radians for ROS
        double yaw_rad = yaw_ * M_PI / 180.0;
        msg.command_values = {surge_, yaw_rad};
    }

    publisher_->publish(msg);
}

void TeleopNode::process_key(char c)
{
    if (control_mode_ == CONTROL_MODE_RPM) {
        // RPM control mode
        switch (c) {
        case 'w':
            left_rpm_ += rpm_increment_;
            right_rpm_ += rpm_increment_;
            break;
        case 's':
            left_rpm_ -= rpm_increment_;
            right_rpm_ -= rpm_increment_;
            break;
        case 'a':
            left_rpm_ -= rpm_increment_;
            right_rpm_ += rpm_increment_;
            break;
        case 'd':
            left_rpm_ += rpm_increment_;
            right_rpm_ -= rpm_increment_;
            break;
        case ' ':
            left_rpm_ = 0.0;
            right_rpm_ = 0.0;
            break;
        case 'q':
            RCLCPP_INFO(this->get_logger(), "Quitting teleop...");
            rclcpp::shutdown();
            return;
        default:
            return;  // Don't log for unrecognized keys
        }

        // Clamp RPM values
        left_rpm_ = std::max(-max_rpm_, std::min(max_rpm_, left_rpm_));
        right_rpm_ = std::max(-max_rpm_, std::min(max_rpm_, right_rpm_));

        // Log current RPM values
        RCLCPP_INFO(this->get_logger(), "RPM - Left: %.1f, Right: %.1f", left_rpm_, right_rpm_);

    } else if (control_mode_ == CONTROL_MODE_SURGE_YAW) {
        // Surge/Yaw control mode
        switch (c) {
        case 'w':
            surge_ += surge_increment_;
            break;
        case 's':
            surge_ -= surge_increment_;
            break;
        case 'a':
            yaw_ -= yaw_increment_;
            break;
        case 'd':
            yaw_ += yaw_increment_;
            break;
        case ' ':
            surge_ = 0.0;
            yaw_ = 0.0;
            break;
        case 'q':
            RCLCPP_INFO(this->get_logger(), "Quitting teleop...");
            rclcpp::shutdown();
            return;
        default:
            return;  // Don't log for unrecognized keys
        }

        // Clamp surge (no clamping for yaw as requested)
        surge_ = std::max(-max_surge_, std::min(max_surge_, surge_));

        // Log current surge/yaw values (yaw in degrees)
        RCLCPP_INFO(this->get_logger(), "Surge: %.2f m/s, Yaw: %.1f deg", surge_, yaw_);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}