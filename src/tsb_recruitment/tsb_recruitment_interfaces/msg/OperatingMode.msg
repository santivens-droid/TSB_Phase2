# OperatingMode.msg
#
# This message represents different operational states
# This acts as an enum-like structure in ROS2

# Possible operating modes
uint8 MODE_OFF = 0          # System in standby/idle
uint8 MODE_EMERGENCY = 1    # Killswitch activated or fault detected
uint8 MODE_REMOTE = 2       # Remote operated
uint8 MODE_AUTONOMOUS = 3   # Autonomous operation

# The current operating mode
uint8 mode


# Text description of the operating mode
string mode_text
