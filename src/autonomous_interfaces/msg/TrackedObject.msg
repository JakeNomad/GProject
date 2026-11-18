# Object state estimated by the Kalman filter after sensor fusion.
uint32 id
string class_label
float32 confidence
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size
float64[16] covariance   # 4x4 row-major [x, y, vx, vy]
uint32 age               # number of filter updates
