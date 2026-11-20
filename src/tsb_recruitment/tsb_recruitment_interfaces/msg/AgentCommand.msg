# Generic agent command message for HoloOcean agents
# Supports all control schemes: Thrusters & Fins (0), PD Controller (1), Custom Dynamics (2)

# Header for timestamp and frame information
std_msgs/Header header

# Command values - interpretation depends on control_scheme
# - Scheme 0 (Thrusters & Fins): Variable length based on agent configuration
# - Scheme 1 (PD Controller): Variable length based on agent DOF
# - Scheme 2 (Custom Dynamics): Always 6 values [lin_accel_x, lin_accel_y, lin_accel_z, ang_accel_x, ang_accel_y, ang_accel_z]
float64[] command_values


# ! Actually Scheme 0 does nothing for otter and 1 is rpm control, 2 is PID