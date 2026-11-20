#ifndef SP01_DATA_GENERATOR_NODE_HPP_
#define SP01_DATA_GENERATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tsb_recruitment_interfaces/msg/state.hpp>
#include <tsb_recruitment_interfaces/msg/operating_mode.hpp>
#include <tsb_recruitment_interfaces/srv/set_operating_mode.hpp>

// Topic name defines
#define BATTERY_VOLTAGE_TOPIC "battery/voltage"
#define BATTERY_CURRENT_TOPIC "battery/current"
#define BATTERY_TEMPERATURE_TOPIC "battery/temperature"
#define LEFT_THRUSTER_CMD_TOPIC "left_thruster/cmd_current"
#define RIGHT_THRUSTER_CMD_TOPIC "right_thruster/cmd_current"
#define AHRS_IMU_TOPIC "ahrs/imu"
#define AHRS_VELOCITY_TOPIC "ahrs/velocity"
#define AHRS_GPS_TOPIC "ahrs/gps"
#define STATE_TOPIC "state"
#define OPERATING_MODE_TOPIC "operating_mode"
#define REQUEST_OPERATING_MODE_SERVICE "request_operating_mode"

// Frequency defines (in Hz)
#define BATTERY_DATA_FREQUENCY 5.0
#define THRUSTER_CMD_FREQUENCY 20.0
#define AHRS_DATA_FREQUENCY 50.0
#define GPS_DATA_FREQUENCY 2.0
#define STATE_DATA_FREQUENCY 10.0
#define OPERATING_MODE_FREQUENCY 1.0

// Frame ID defines
#define AHRS_FRAME_ID "ahrs_link"

class SP01DataGeneratorNode : public rclcpp::Node
{
public:
    SP01DataGeneratorNode(const rclcpp::NodeOptions &options);

private:
    void initializeParameters();
    void initializeTransforms();
    void initializePublishers();
    void initializeSubscribers();
    void initializeDiagnostics();
    void initializeTimer();
    void initializeServices();

    void publish_battery_data();
    void publish_thruster_data();
    void publish_ahrs_data();
    void publish_gps_data();
    void publish_state_data();
    void publish_operating_mode_data();

    void set_operating_mode_callback(
        const std::shared_ptr<tsb_recruitment_interfaces::srv::SetOperatingMode::Request> request,
        std::shared_ptr<tsb_recruitment_interfaces::srv::SetOperatingMode::Response> response);

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_temperature_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_thruster_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_thruster_cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ahrs_imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr ahrs_velocity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr ahrs_gps_pub_;
    rclcpp::Publisher<tsb_recruitment_interfaces::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<tsb_recruitment_interfaces::msg::OperatingMode>::SharedPtr operating_mode_pub_;

    // Service
    rclcpp::Service<tsb_recruitment_interfaces::srv::SetOperatingMode>::SharedPtr set_operating_mode_service_;

    // Timers for different frequencies
    rclcpp::TimerBase::SharedPtr battery_timer_;        // 5 Hz - battery monitoring
    rclcpp::TimerBase::SharedPtr thruster_timer_;       // 20 Hz - control commands
    rclcpp::TimerBase::SharedPtr ahrs_timer_;           // 50 Hz - IMU/velocity
    rclcpp::TimerBase::SharedPtr gps_timer_;            // 2 Hz - GPS updates
    rclcpp::TimerBase::SharedPtr state_timer_;          // 10 Hz - state estimation
    rclcpp::TimerBase::SharedPtr operating_mode_timer_; // 1 Hz - mode status

    // Current operating mode
    tsb_recruitment_interfaces::msg::OperatingMode current_mode_;
};

#endif  // SP01_EMULATOR_NODE_HPP_
