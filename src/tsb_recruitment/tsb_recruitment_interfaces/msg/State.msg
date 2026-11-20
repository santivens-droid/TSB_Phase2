# State.msg
#
# This message holds the state of the vehicle.
# It includes the position in GPS and UTM coordinates
# the orientation in roll, pitch and yaw, the dynamics variables (surge, sway, heave)

# Standard ROS header
std_msgs/Header header

# Latitude and Longitude
tsb_recruitment_interfaces/GPSPoint gps

# UTM Position
tsb_recruitment_interfaces/UTMPoint utm

# Orientation
float32 roll
float32 pitch
float32 yaw

# Dynamics Variables
float32 surge
float32 sway
float32 heave


# TODO: Tirar isto daqui. Os recrutas de Position and Orientation control podem fazer isto
