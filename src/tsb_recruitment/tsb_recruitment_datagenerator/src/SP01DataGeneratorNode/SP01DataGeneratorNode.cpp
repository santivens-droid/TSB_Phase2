#include <tsb_recruitment_datagenerator/SP01DataGeneratorNode.hpp>

SP01DataGeneratorNode::SP01DataGeneratorNode(const rclcpp::NodeOptions &options)
    : Node("sp01_datagenerator_node", options)
{
    initializeParameters();
    initializeTransforms();
    initializePublishers();
    initializeSubscribers();
    initializeDiagnostics();
    initializeTimer();
    initializeServices();

    RCLCPP_INFO(this->get_logger(), "Node initialized.");
}

void SP01DataGeneratorNode::initializeParameters()
{
    // Initialize parameters if needed
}

void SP01DataGeneratorNode::initializeTransforms()
{
    // Initialize transforms if needed
}

void SP01DataGeneratorNode::initializePublishers()
{
    battery_voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>(BATTERY_VOLTAGE_TOPIC, 10);
    battery_current_pub_ = this->create_publisher<std_msgs::msg::Float32>(BATTERY_CURRENT_TOPIC, 10);
    battery_temperature_pub_ = this->create_publisher<std_msgs::msg::Float32>(BATTERY_TEMPERATURE_TOPIC, 10);
    left_thruster_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32>(LEFT_THRUSTER_CMD_TOPIC, 10);
    right_thruster_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32>(RIGHT_THRUSTER_CMD_TOPIC, 10);
    ahrs_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(AHRS_IMU_TOPIC, 10);
    ahrs_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(AHRS_VELOCITY_TOPIC, 10);
    ahrs_gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(AHRS_GPS_TOPIC, 10);
    state_pub_ = this->create_publisher<tsb_recruitment_interfaces::msg::State>(STATE_TOPIC, 10);
    operating_mode_pub_ = this->create_publisher<tsb_recruitment_interfaces::msg::OperatingMode>(OPERATING_MODE_TOPIC, 10);
}

void SP01DataGeneratorNode::initializeSubscribers()
{
    // No subscribers needed for this emulator
}

void SP01DataGeneratorNode::initializeDiagnostics()
{
    // Initialize diagnostics if needed
}

void SP01DataGeneratorNode::initializeTimer()
{
    // Battery data at BATTERY_DATA_FREQUENCY Hz
    battery_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / BATTERY_DATA_FREQUENCY)),
        std::bind(&SP01DataGeneratorNode::publish_battery_data, this));

    // Thruster commands at THRUSTER_CMD_FREQUENCY Hz
    thruster_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / THRUSTER_CMD_FREQUENCY)),
        std::bind(&SP01DataGeneratorNode::publish_thruster_data, this));

    // AHRS data at AHRS_DATA_FREQUENCY Hz
    ahrs_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / AHRS_DATA_FREQUENCY)),
        std::bind(&SP01DataGeneratorNode::publish_ahrs_data, this));

    // GPS data at GPS_DATA_FREQUENCY Hz
    gps_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / GPS_DATA_FREQUENCY)),
        std::bind(&SP01DataGeneratorNode::publish_gps_data, this));

    // State data at STATE_DATA_FREQUENCY Hz
    state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / STATE_DATA_FREQUENCY)),
        std::bind(&SP01DataGeneratorNode::publish_state_data, this));

    // Operating mode at OPERATING_MODE_FREQUENCY Hz
    operating_mode_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / OPERATING_MODE_FREQUENCY)),
        std::bind(&SP01DataGeneratorNode::publish_operating_mode_data, this));
}

void SP01DataGeneratorNode::initializeServices()
{
    set_operating_mode_service_ = this->create_service<tsb_recruitment_interfaces::srv::SetOperatingMode>(
        REQUEST_OPERATING_MODE_SERVICE,
        std::bind(&SP01DataGeneratorNode::set_operating_mode_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize operating mode
    current_mode_.mode = tsb_recruitment_interfaces::msg::OperatingMode::MODE_REMOTE;
    current_mode_.mode_text = "Remote operation";
}

void SP01DataGeneratorNode::publish_battery_data()
{
    auto now = this->get_clock()->now();

    // Publish battery data at 5 Hz
    // Battery Voltage: 24V nominal LiPo battery pack, oscillating ±2V around nominal
    auto voltage_msg = std_msgs::msg::Float32();
    voltage_msg.data = 24.0 + 2.0 * sin(now.seconds());
    battery_voltage_pub_->publish(voltage_msg);

    // Battery Current: 10A average draw with ±5A variation
    auto current_msg = std_msgs::msg::Float32();
    current_msg.data = 10.0 + 5.0 * cos(now.seconds());
    battery_current_pub_->publish(current_msg);

    // Battery Temperature: 25°C ambient with ±5°C variation
    auto temp_msg = std_msgs::msg::Float32();
    temp_msg.data = 25.0 + 5.0 * sin(now.seconds() * 0.5);
    battery_temperature_pub_->publish(temp_msg);
}

void SP01DataGeneratorNode::publish_thruster_data()
{
    auto now = this->get_clock()->now();

    // Publish thruster commands at 20 Hz
    // Thruster Current Commands: 5A base current with ±2A variation for propulsion control
    // Realistic range: 0-20A per thruster (depending on motor size and water conditions)
    // Positive values = forward thrust, negative = reverse (if supported)
    auto left_cmd = std_msgs::msg::Float32();
    left_cmd.data = 5.0 + 2.0 * sin(now.seconds());
    left_thruster_cmd_pub_->publish(left_cmd);

    auto right_cmd = std_msgs::msg::Float32();
    right_cmd.data = 5.0 + 2.0 * cos(now.seconds());
    right_thruster_cmd_pub_->publish(right_cmd);
}

void SP01DataGeneratorNode::publish_ahrs_data()
{
    auto now = this->get_clock()->now();

    // Publish AHRS/IMU data at 50 Hz
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = AHRS_FRAME_ID;

    // Linear Acceleration (m/s²) in NED frame - small boat motions + gravity
    // Realistic ranges: ±2-5 m/s² for normal boat motions, Z always includes gravity (-9.81)
    imu_msg.linear_acceleration.x = 0.1 * sin(now.seconds());     // Surge acceleration
    imu_msg.linear_acceleration.y = 0.1 * cos(now.seconds());     // Sway acceleration
    imu_msg.linear_acceleration.z = -9.81;                        // Gravity + heave acceleration

    // Angular Velocity (rad/s) - boat rotation rates
    // Realistic ranges: ±0.5-2 rad/s for normal maneuvering, higher in rough conditions
    imu_msg.angular_velocity.x = 0.01 * sin(now.seconds());       // Roll rate
    imu_msg.angular_velocity.y = 0.01 * cos(now.seconds());       // Pitch rate
    imu_msg.angular_velocity.z = 0.01 * sin(now.seconds() * 2);   // Yaw rate

    ahrs_imu_pub_->publish(imu_msg);

    // Velocity estimates from AHRS (m/s) in body frame
    // Realistic ranges: 0-5 m/s forward speed, ±1 m/s lateral, ±0.2 m/s vertical (wave motion)
    auto velocity_msg = geometry_msgs::msg::Vector3Stamped();
    velocity_msg.header.stamp = now;
    velocity_msg.header.frame_id = AHRS_FRAME_ID;
    velocity_msg.vector.x = 1.0 + 0.5 * sin(now.seconds());  // Forward velocity (surge)
    velocity_msg.vector.y = 0.1 * cos(now.seconds());       // Lateral velocity (sway)
    velocity_msg.vector.z = 0.05 * sin(now.seconds() * 3);  // Vertical velocity (heave - wave motion)
    ahrs_velocity_pub_->publish(velocity_msg);
}

void SP01DataGeneratorNode::publish_gps_data()
{
    auto now = this->get_clock()->now();

    // Publish GPS data at 2 Hz
    // GPS coordinates: Lisbon harbor area with small position variations
    // Latitude/Longitude: WGS84 coordinates, Lisbon Portugal area
    // Realistic accuracy: ±2-5 meters with small sinusoidal drift simulation
    auto gps_msg = sensor_msgs::msg::NavSatFix();
    gps_msg.header.stamp = now;
    gps_msg.header.frame_id = AHRS_FRAME_ID;
    gps_msg.latitude = 38.7369 + 0.001 * sin(now.seconds() * 0.1);   // ~111m variation
    gps_msg.longitude = -9.1393 + 0.001 * cos(now.seconds() * 0.1);  // ~78m variation at this latitude
    // Simulate altitude with tides (slow), waves (faster), and small sensor jitter
    const double two_pi = 2.0 * 3.141592653589793;
    double tide = 0.5 * sin(now.seconds() * two_pi / 43200.0);   // ±0.5 m tidal variation (≈12h period)
    double waves = 0.2 * sin(now.seconds() * two_pi / 10.0);     // ±0.2 m waves (~10s period)
    double jitter = 0.05 * sin(now.seconds() * 37.0);            // small sensor noise
    gps_msg.altitude = tide + waves + jitter;                    // altitude in meters (MSL)

    ahrs_gps_pub_->publish(gps_msg);
}

void SP01DataGeneratorNode::publish_state_data()
{
    auto now = this->get_clock()->now();

    // Publish state data at 10 Hz (fused sensor data for navigation)
    auto state_msg = tsb_recruitment_interfaces::msg::State();
    state_msg.header.stamp = now;
    state_msg.header.frame_id = "base_link";

    // GPS Position: Current position from GPS sensor
    state_msg.gps.latitude = 38.7369 + 0.001 * sin(now.seconds() * 0.1);
    state_msg.gps.longitude = -9.1393 + 0.001 * cos(now.seconds() * 0.1);

    // UTM Position: Would be converted from GPS in real system
    // For simulation, keeping at origin with Lisbon UTM zone
    state_msg.utm.easting = 0.0;     // Easting in meters from zone origin
    state_msg.utm.northing = 0.0;   // Northing in meters from equator
    state_msg.utm.zone = 29;         // UTM zone for Lisbon area
    state_msg.utm.north = true;      // Northern hemisphere

    // Orientation (Euler angles in radians)
    // Realistic ranges: ±30° roll/pitch in rough seas, continuous yaw
    state_msg.roll = 0.01 * sin(now.seconds()) * 10;     // ±0.1 radians (~5.7°)
    state_msg.pitch = 0.01 * cos(now.seconds()) * 10;    // ±0.1 radians (~5.7°)
    state_msg.yaw = 0.01 * sin(now.seconds() * 2) * 10;  // ±0.1 radians (~5.7°)

    // Dynamics (velocities in m/s, body frame)
    // Realistic ranges: 0-3 m/s surge, ±0.5 m/s sway, minimal heave
    state_msg.surge = 1.0 + 0.5 * sin(now.seconds());    // Forward speed
    state_msg.sway = 0.1 * cos(now.seconds());           // Lateral speed
    state_msg.heave = 0.0;                               // Vertical speed (minimal)

    state_pub_->publish(state_msg);
}

void SP01DataGeneratorNode::publish_operating_mode_data()
{
    // Publish operating mode at 1 Hz
    // Operating modes:
    // 0 = MODE_OFF: System powered down/safe state
    // 1 = MODE_EMERGENCY: Emergency stop activated (highest priority)
    // 2 = MODE_REMOTE: Remote control operation (default for safety)
    // 3 = MODE_AUTONOMOUS: Autonomous operation with AI control
    operating_mode_pub_->publish(current_mode_);
}

void SP01DataGeneratorNode::set_operating_mode_callback(
    const std::shared_ptr<tsb_recruitment_interfaces::srv::SetOperatingMode::Request> request,
    std::shared_ptr<tsb_recruitment_interfaces::srv::SetOperatingMode::Response> response)
{
    response->previous_mode = current_mode_;
    current_mode_ = request->mode;
    response->new_mode = current_mode_;
    response->success = true;

    RCLCPP_INFO(this->get_logger(), "Operating mode changed from %s to %s",
               response->previous_mode.mode_text.c_str(),
               response->new_mode.mode_text.c_str());
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<SP01DataGeneratorNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}