# UIPopup.msg
#
# Message for displaying popup notifications on the user interface.
# When published, this will trigger a popup display for the specified duration.

# Standard ROS header
std_msgs/Header header

# Severity levels
uint8 LEVEL_DEBUG = 0      # Debug information
uint8 LEVEL_INFO = 1       # General information
uint8 LEVEL_SUCCESS = 2    # Success confirmation
uint8 LEVEL_WARNING = 3    # Warning message
uint8 LEVEL_ERROR = 4      # Error condition
uint8 LEVEL_CRITICAL = 5   # Critical system issue

# Popup title
string title

# Popup description/message
string description

# Severity level (use the constants above)
uint8 level

# Time in seconds to display the popup (0 = manual dismiss only)
float32 display_duration