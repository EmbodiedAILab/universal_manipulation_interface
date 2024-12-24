import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import zmq
import pickle

class ROStoZMQBridge(Node):
    def __init__(self):
        super().__init__('ros_zmq_bridge_camera')

        zmq_host = self.declare_parameter('zmq_host', '127.0.0.1').get_parameter_value().string_value
        zmq_port = self.declare_parameter('zmq_port', '5566').get_parameter_value().string_value
        ros_topic = self.declare_parameter('ros_topic', '/zed_camera_image_color').get_parameter_value().string_value
        zmq_message_name = self.declare_parameter('zmq_message_name', 'camera_0').get_parameter_value().string_value

        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{zmq_host}:{zmq_port}")

        self.bridge = CvBridge()

        self.create_subscription(Image, ros_topic, self.image_callback, 10)

        self.zmq_message_name = zmq_message_name
        self.get_logger().info(f"Bridge initialized. ZMQ publishing to tcp://{zmq_host}:{zmq_port}, listening to ROS topic '{ros_topic}'.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        # 序列化图像数据
        serialized_data = pickle.dumps(cv_image)

        try:
            message = self.zmq_message_name.encode('utf-8') + b' ' + serialized_data
            self.socket.send(message)
            self.get_logger().debug(f"Published image with topic '{self.zmq_message_name}'.")
        except Exception as e:
            self.get_logger().error(f"Error sending data over ZeroMQ: {e}")

def main(args=None):
    rclpy.init(args=args)
    ros2zmq_bridge_node = ROStoZMQBridge()
    
    try:
        rclpy.spin(ros2zmq_bridge_node)
    except KeyboardInterrupt:
        ros2zmq_bridge_node.get_logger().info("Shutting down bridge.")
    finally:
        ros2zmq_bridge_node.destroy_node()
        ros2zmq_bridge_node.get_logger().info("ZeroMQ context terminated.")
        rclpy.shutdown()

if __name__ == "__main__":
    main()