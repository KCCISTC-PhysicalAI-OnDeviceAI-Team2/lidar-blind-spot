# One obstacle on the floor
string  label        # class name. "unknown" for LiDAR
float32 x            # floor position in base_jig, forward +x [m]
float32 y            # left +y [m]
float32 confidence   # 0-1. 1.0 for LiDAR
string  source       # "lidar" or "camera"