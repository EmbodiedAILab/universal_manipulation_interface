import zmq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import time
import msgpack

class ZMQtoROSBridge(Node):
    def __init__(self, zmq_host="127.0.0.1", zmq_port='5554'):
        super().__init__('zmq_to_ros_bridge')

        zmq_host = self.declare_parameter('zmq_host', '127.0.0.1').get_parameter_value().string_value
        zmq_port = self.declare_parameter('zmq_port', '5554').get_parameter_value().string_value
        servoL_topic_name = self.declare_parameter('servoL_topic_name', '/eef_servo_controller').get_parameter_value().string_value
        moveJ_topic_name = self.declare_parameter('moveJ_topic_name', '/moveJ_cmd').get_parameter_value().string_value

        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{zmq_host}:{zmq_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self.servoL_pub = self.create_publisher(PoseStamped, servoL_topic_name, 10)
        self.moveJ_pub = self.create_publisher(JointState, moveJ_topic_name, 10)

    def zmq_to_ros(self):
        while rclpy.ok():
            try:
                topic, serialized_data = self.socket.recv_multipart()
                topic = topic.decode('utf-8')  

                print(f"Received topic: {topic}")
                data = msgpack.unpackb(serialized_data, raw=False)
                print(f"Received data: {data}")

                if topic == "servoL_cmd":
                    pose_data = data  
                    pose_stamped = PoseStamped()
                    pose_stamped.pose.position.x = pose_data['position']['x']
                    pose_stamped.pose.position.y = pose_data['position']['y']
                    pose_stamped.pose.position.z = pose_data['position']['z']
                    pose_stamped.pose.orientation.x = pose_data['orientation']['x']
                    pose_stamped.pose.orientation.y = pose_data['orientation']['y']
                    pose_stamped.pose.orientation.z = pose_data['orientation']['z']
                    pose_stamped.pose.orientation.w = pose_data['orientation']['w']
                    timestamp = pose_data['timestamp']
                    pose_stamped.header.stamp.sec = timestamp // 1_000_000_000
                    pose_stamped.header.stamp.nanosec = timestamp % 1_000_000_000
                    self.get_logger().info(f"/servoL_cmd timestamp: {timestamp}")
                    self.get_logger().info(f"[ZMQ2ROS] latency: {(time.time_ns() - timestamp) / 1e9}")
                                        
                    self.servoL_pub.publish(pose_stamped)
                    self.get_logger().info(f"Published /servoL_cmd: {pose_stamped}")
                elif topic == "moveJ_cmd":
                    joint_data = data  
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = [f'joint_{i}' for i in range(len(joint_data['positions']))]
                    joint_state_msg.position = joint_data['positions']
                    self.moveJ_pub.publish(joint_state_msg)
                    self.get_logger().info(f"Published /moveJ_cmd: {joint_data['positions']}")
                else:
                    self.get_logger().warn(f"Unknown topic: {topic}")
            except Exception as e:
                self.get_logger().error(f"Error receiving data: {e}")
    
    def destroy(self):
        self.socket.close()
        self.zmq_context.term()

def main(args=None):
    rclpy.init(args=args)
    zmq2ros_bridge_node = ZMQtoROSBridge()
    
    try:
        zmq2ros_bridge_node.zmq_to_ros()
    except KeyboardInterrupt:
        zmq2ros_bridge_node.get_logger().info("Shutting down bridge.")
    finally:
        zmq2ros_bridge_node.destroy()
        zmq2ros_bridge_node.get_logger().info("ZeroMQ context terminated.")
        rclpy.shutdown()

if __name__ == "__main__":
    main()