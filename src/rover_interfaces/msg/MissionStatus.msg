std_msgs/Header header
string mission_mode      # TELEOP, AUTONOMOUS, ABEX, IDMO, EMERGENCY_STOP
string current_state     # IDLE, RECON, NAVIGATING_TO_OBJECT, PICKING_OBJECT, NAVIGATING_TO_GOAL, DELIVERING, RECOVERING
string active_target     # Name/ID of current target object or location
float32 battery_level    # 0-100 percentage
float32 mission_time_sec # Elapsed mission time in seconds
bool autonomy_locked     # True if manual command lock is active
