std_msgs/Header header
bool cliff_detected       # True if a cliff/drop is detected
float32 distance_to_drop  # Meters to detected drop edge
float32 drop_depth_meters # Depth of vertical drop
geometry_msgs/Point cliff_position # Position relative to base_link
