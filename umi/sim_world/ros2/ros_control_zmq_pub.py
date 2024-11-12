import zmq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import pickle

class ZMQtoROSBridge(Node):
    def __init__(self, zmq_host="localhost", zmq_port=5554):
        super().__init__('zmq_to_ros_bridge')

        # 初始化 ZeroMQ 上下文和套接字
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{zmq_host}:{zmq_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # 订阅所有消息

        # ROS 发布者
        self.servoL_pub = self.create_publisher(Pose, '/servoL_cmd', 10)
        self.moveJ_pub = self.create_publisher(JointState, '/moveJ_cmd', 10)
        self.gripper_pub = self.create_publisher(JointState, '/gripper_cmd', 10)

    def zmq_to_ros(self):
        while rclpy.ok():
            try:
                topic, serialized_data = self.socket.recv_multipart()
                topic = topic.decode('utf-8')  # 将 topic 从字节字符串转换为普通字符串

                # topic, serialized_data = self.socket.recv_multipart()
                print(f"Received topic: {topic}")
                data = pickle.loads(serialized_data)
                print(f"Received data: {data}")

                # 根据 ZeroMQ 主题名称选择发布 ROS 话题
                if topic == "servoL_cmd":
                    pose_data = data  # 确保数据是字典格式
                    pose = Pose()
                    pose.position.x = pose_data['position']['x']
                    pose.position.y = pose_data['position']['y']
                    pose.position.z = pose_data['position']['z']
                    pose.orientation.x = pose_data['orientation']['x']
                    pose.orientation.y = pose_data['orientation']['y']
                    pose.orientation.z = pose_data['orientation']['z']
                    pose.orientation.w = pose_data['orientation']['w']
                    
                    self.servoL_pub.publish(pose)
                    self.get_logger().info(f"Published /servoL_cmd: {pose}")
                elif topic == "moveJ_cmd":
                    joint_data = data  # 确保数据是字典格式
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = [f'joint_{i}' for i in range(len(joint_data['positions']))]
                    joint_state_msg.position = joint_data['positions']
                    self.moveJ_pub.publish(joint_state_msg)
                    self.get_logger().info(f"Published /moveJ_cmd: {joint_data['positions']}")
                elif topic == "gripper_cmd":
                    gripper_data = data  # 确保数据是字典格式
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = ["gripper"]
                    joint_state_msg.position = gripper_data['positions']
                    self.gripper_pub.publish(joint_state_msg)
                    self.get_logger().info(f"Published /gripper_cmd: {gripper_data['positions']}")
                else:
                    self.get_logger().warn(f"Unknown topic: {topic}")
            except Exception as e:
                self.get_logger().error(f"Error receiving data: {e}")

    def destroy(self):
        self.socket.close()
        self.zmq_context.term()


def main():
    rclpy.init()
    bridge_node = ZMQtoROSBridge()
    try:
        bridge_node.zmq_to_ros()
    except KeyboardInterrupt:
        bridge_node.get_logger().info("Shutting down bridge.")
    finally:
        bridge_node.destroy()
        bridge_node.get_logger().info("ZeroMQ context terminated.")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
