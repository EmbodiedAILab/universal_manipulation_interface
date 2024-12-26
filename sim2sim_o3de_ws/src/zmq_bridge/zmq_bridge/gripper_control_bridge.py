import zmq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import msgpack

class GripperZMQtoROSBridge(Node):
    def __init__(self, zmq_host="127.0.0.1", zmq_port='5556'):
        super().__init__('zmq_to_ros_bridge')

        zmq_host = self.declare_parameter('zmq_host', '127.0.0.1').get_parameter_value().string_value
        zmq_port = self.declare_parameter('zmq_port', '5556').get_parameter_value().string_value
        gripper_control_topic_name = self.declare_parameter('gripper_control_topic_name', '/gripper_cmd').get_parameter_value().string_value

        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{zmq_host}:{zmq_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self.gripper_pub = self.create_publisher(JointState, gripper_control_topic_name, 10)

    def zmq_to_ros(self):
        while rclpy.ok():
            try:
                topic, serialized_data = self.socket.recv_multipart()
                topic = topic.decode('utf-8')  
                data = msgpack.unpackb(serialized_data, raw=False)

                if topic == "gripper_cmd":
                    gripper_data = data  
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = ["gripper"]
                    joint_state_msg.position = [gripper_data['positions']]
                    self.gripper_pub.publish(joint_state_msg)
                    self.get_logger().info(f"Published /gripper_cmd: {gripper_data['positions']}")
                else:
                    self.get_logger().warn(f"Unknown topic: {topic}")
            except Exception as e:
                self.get_logger().error(f"Error receiving data: {e}")

    def destroy(self):
        self.socket.close()
        self.zmq_context.term()

def main(args=None):
    rclpy.init(args=args)
    gripper_cmd_node = GripperZMQtoROSBridge()
    
    try:
        gripper_cmd_node.zmq_to_ros()
    except KeyboardInterrupt:
        gripper_cmd_node.get_logger().info("Shutting down bridge.")
    finally:
        gripper_cmd_node.destroy()
        gripper_cmd_node.get_logger().info("ZeroMQ context terminated.")
        rclpy.shutdown()

if __name__ == "__main__":
    main()