uint8 LANE_FOLLOW=0
uint8 FOLLOW_VEHICLE=1
uint8 STOP=2
uint8 EMERGENCY_BRAKE=3
uint8 LANE_CHANGE_LEFT=4
uint8 LANE_CHANGE_RIGHT=5

std_msgs/Header header
uint8 state
float32 target_speed   # [m/s]
uint32 lead_object_id  # 0 if none
