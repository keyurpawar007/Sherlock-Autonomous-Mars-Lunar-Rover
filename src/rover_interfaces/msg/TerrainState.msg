std_msgs/Header header
float32 slope_angle_deg   # Terrain slope angle in degrees
float32 roughness_score   # Terrain roughness metric (0 = smooth, 1 = extremely rough)
bool is_traversable       # True if terrain angle and roughness are safe
string terrain_type       # FLAT, SLOPE, ROUGH, NO_GO
