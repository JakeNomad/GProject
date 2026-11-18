# Single object detected by one sensor modality (camera / lidar / radar).
uint8 SOURCE_CAMERA=0
uint8 SOURCE_LIDAR=1
uint8 SOURCE_RADAR=2

uint8 source
string class_label            # e.g. "car", "pedestrian", "cyclist"
float32 confidence            # [0, 1]
geometry_msgs/Point position  # base_link [m]; camera: unit bearing vector (no range)
geometry_msgs/Vector3 size    # length, width, height [m] (if known)
vision_msgs/BoundingBox2D bbox_2d  # image-space box (camera only)
float32 radial_velocity     # [m/s] radar only, negative = approaching
