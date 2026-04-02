import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('input_topic', '/depth_camera/image')
        self.declare_parameter('output_topic', '/depth_camera/image_detected')

        model_name = self.get_parameter('model').get_parameter_value().string_value
        self.conf = self.get_parameter('confidence').get_parameter_value().double_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.get_logger().info(f'Cargando modelo YOLO: {model_name}')
        self.model = YOLO(model_name)
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, input_topic, self.callback, 10)
        self.pub = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(
            f'YoloDetector activo | {input_topic} → {output_topic} | conf={self.conf}'
        )

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame, conf=self.conf, verbose=False)

        annotated = results[0].plot(
            line_width=2,
            font_size=0.6,
        )

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
