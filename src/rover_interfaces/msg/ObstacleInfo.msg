std_msgs/Header header

bool detected
bool surety
uint8 confidence_pct

float32 distance_m
float32 width_m
float32 height_m

# Obstacle position relative to the sensor/robot.
# forward_m: distance forward
# lateral_m: left/right displacement
float32 forward_m
float32 lateral_m

# Perception classification
uint8 action_code
string decision
