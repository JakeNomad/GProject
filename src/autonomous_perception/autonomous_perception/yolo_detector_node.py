"""Camera object detection with a YOLO model.

Subscribed:  /carla/ego/camera/front/image_raw    sensor_msgs/Image
             /carla/ego/camera/front/camera_info  sensor_msgs/CameraInfo
Published:   /perception/camera/detections        autonomous_interfaces/DetectedObjectArray
             /perception/camera/debug_image       sensor_msgs/Image (annotated, optional)

Camera detections carry the 2D box plus a unit bearing vector in
`position` (x forward, y left, z=0, base_link axes); range is resolved later
in the fusion layer by associating with LiDAR/radar objects.
"""
import math

from autonomous_interfaces.msg import DetectedObject, DetectedObjectArray
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

# COCO class ids that matter for driving
DRIVING_CLASSES = {0: 'pedestrian', 1: 'cyclist', 2: 'car', 3: 'motorcycle',
                   5: 'bus', 7: 'truck', 9: 'traffic_light', 11: 'stop_sign'}


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('device', 'cpu')  # 'cuda:0' for GPU
        self.declare_parameter('publish_debug_image', True)

        self.bridge = CvBridge()
        self.fx = None
        self.cx = None
        self.model = None
        if YOLO is None:
            self.get_logger().error('ultralytics not installed: pip install ultralytics')
        else:
            self.model = YOLO(self.get_parameter('model_path').value)

        self.det_pub = self.create_publisher(
            DetectedObjectArray, '/perception/camera/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/camera/debug_image', 1)
        self.create_subscription(
            CameraInfo, '/carla/ego/camera/front/camera_info', self.on_info, 1)
        self.create_subscription(
            Image, '/carla/ego/camera/front/image_raw', self.on_image, 1)

    def on_info(self, info):
        self.fx, self.cx = info.k[0], info.k[2]

    def on_image(self, msg):
        if self.model is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self.model.predict(
            frame, conf=self.get_parameter('confidence_threshold').value,
            device=self.get_parameter('device').value, verbose=False)[0]

        out = DetectedObjectArray(header=msg.header)
        for box in result.boxes:
            cls = int(box.cls)
            if cls not in DRIVING_CLASSES:
                continue
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            obj = DetectedObject(source=DetectedObject.SOURCE_CAMERA,
                                 class_label=DRIVING_CLASSES[cls], confidence=float(box.conf))
            obj.bbox_2d.center.position.x = (x1 + x2) / 2.0
            obj.bbox_2d.center.position.y = (y1 + y2) / 2.0
            obj.bbox_2d.size_x = x2 - x1
            obj.bbox_2d.size_y = y2 - y1
            if self.fx:
                bearing = math.atan2(self.cx - obj.bbox_2d.center.position.x, self.fx)
                obj.position.x, obj.position.y = math.cos(bearing), math.sin(bearing)
            out.objects.append(obj)
        self.det_pub.publish(out)

        if self.get_parameter('publish_debug_image').value:
            dbg = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
            dbg.header = msg.header
            self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
