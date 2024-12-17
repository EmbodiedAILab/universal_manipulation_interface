import zmq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import msgpack

class ROStoZMQBridge(Node):
    def __init__(self, zmq_host="127.0.0.1", zmq_port='5555'):
        super().__init__('ros_zmq_bridge')

        zmq_host = self.declare_parameter('zmq_host', '127.0.0.1').get_parameter_value().string_value
        zmq_port = self.declare_parameter('zmq_port', '5555').get_parameter_value().string_value
        eef_pose_topic = self.declare_parameter('eef_pose_topic', '/eef_pose').get_parameter_value().string_value
        gripper_width_topic = self.declare_parameter('gripper_width_topic', '/gripper_width').get_parameter_value().string_value

        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{zmq_host}:{zmq_port}")

        self.bridge = CvBridge()
        
        self.create_subscription(PoseStamped, eef_pose_topic, self.eef_pose_callback, 10)
        self.create_subscription(Float64, gripper_width_topic, self.gripper_width_callback, 10)

    def eef_pose_callback(self, msg):
        current_time_ns = self.get_clock().now().nanoseconds
        msg_time_ns = msg.header.stamp.sec * int(1e9) + msg.header.stamp.nanosec
        time_diff_sec = (current_time_ns - msg_time_ns) / 1e9

        eef_pose_data = {
            'position': {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z},
            'orientation': {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y, 'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w},
            'timestamp': msg_time_ns
        }
        self.get_logger().info(
            f"[ROS2ZMQ] eef_pose msg generated {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, "
            f"received {current_time_ns / 1e9:.9f}, time difference: {time_diff_sec:.9f} seconds"
        )

        serialized_data = msgpack.packb(eef_pose_data, use_bin_type=True)
        self.socket.send_multipart([b'eef_pose', serialized_data])
        self.get_logger().info(f"Sent eef_pose to ZMQ: {eef_pose_data}")

    def gripper_width_callback(self, msg):
        gripper_width_data = {'width': msg.data, 'timestamp': self.get_clock().now().nanoseconds}
        serialized_data = msgpack.packb(gripper_width_data, use_bin_type=True)
        self.socket.send_multipart([b'gripper_width', serialized_data])
        self.get_logger().info(f"Sent gripper_width to ZMQ: {gripper_width_data}")

    def destroy(self):
        self.socket.close()
        self.zmq_context.term()

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