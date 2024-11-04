import zmq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import pickle
from cv_bridge import CvBridge
import threading

UR5E_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

class ROStoZMQBridge(Node):
    def __init__(self, zmq_host="localhost", zmq_port=5555):
        super().__init__('ros_zmq_bridge')

        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{zmq_host}:{zmq_port}")

        self.bridge = CvBridge()
        
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(Pose, '/eef_pose', self.eef_pose_callback, 10)
        self.create_subscription(Float64, '/gripper_width', self.gripper_width_callback, 10)

    def image_callback(self, msg):
        # 将 ROS 2 图像消息转换为 OpenCV 图像
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print(f"Failed to convert image: {e}")
            return
        
        # 序列化图像数据
        serialized_data = pickle.dumps(cv_image)

        try:
            message = b'image_topic ' + serialized_data
            self.socket.send(message)
            print(f"Published image with topic 'image_topic'")
        except Exception as e:
            print(f"Error sending data over ZeroMQ: {e}")

    def joint_states_callback(self, msg):
        joint_positions = [0.0] * 6
        for i, joint_name in enumerate(msg.name):
            if joint_name in UR5E_JOINTS:
                index = UR5E_JOINTS.index(joint_name)
                joint_positions[index] = msg.position[i]

        joint_state_data = {'positions': joint_positions, 'timestamp': self.get_clock().now().nanoseconds}
        serialized_data = pickle.dumps(joint_state_data)
        self.socket.send_multipart([b'joint_states', serialized_data])
        self.get_logger().info(f"Sent joint_states to ZMQ: {joint_state_data}")

    def eef_pose_callback(self, msg):
        eef_pose_data = {
            'position': {'x': msg.position.x, 'y': msg.position.y, 'z': msg.position.z},
            'orientation': {'x': msg.orientation.x, 'y': msg.orientation.y, 'z': msg.orientation.z, 'w': msg.orientation.w},
            'timestamp': self.get_clock().now().nanoseconds
        }

        serialized_data = pickle.dumps(eef_pose_data)
        self.socket.send_multipart([b'eef_pose', serialized_data])
        self.get_logger().info(f"Sent eef_pose to ZMQ: {eef_pose_data}")

    def gripper_width_callback(self, msg):
        gripper_width_data = {'width': msg.data, 'timestamp': self.get_clock().now().nanoseconds}
        serialized_data = pickle.dumps(gripper_width_data)
        self.socket.send_multipart([b'gripper_width', serialized_data])
        self.get_logger().info(f"Sent gripper_width to ZMQ: {gripper_width_data}")

    def destroy(self):
        self.socket.close()
        self.zmq_context.term()
        
class ZMQtoROSBridge(Node):
    def __init__(self, zmq_host="localhost", zmq_port=5554):
        super().__init__('zmq_to_ros_bridge')

        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{zmq_host}:{zmq_port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # 订阅所有消息

        self.servoL_pub = self.create_publisher(Pose, '/eef_servo_controller', 10)
        self.moveJ_pub = self.create_publisher(JointState, '/moveJ_cmd', 10)
        self.gripper_pub = self.create_publisher(JointState, '/gripper_cmd', 10)

    def zmq_to_ros(self):
        while rclpy.ok():
            try:
                topic, serialized_data = self.socket.recv_multipart()
                topic = topic.decode('utf-8')  

                print(f"Received topic: {topic}")
                data = pickle.loads(serialized_data)
                print(f"Received data: {data}")

                # 根据 ZeroMQ 主题名称选择发布 ROS 话题
                if topic == "servoL_cmd":
                    pose_data = data
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
                    joint_data = data  
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = [f'joint_{i}' for i in range(len(joint_data['positions']))]
                    joint_state_msg.position = joint_data['positions']
                    self.moveJ_pub.publish(joint_state_msg)
                    self.get_logger().info(f"Published /moveJ_cmd: {joint_data['positions']}")
                elif topic == "gripper_cmd":
                    gripper_data = data  
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

def main(args=None):
    rclpy.init(args=args)
    
    ros2zmq_bridge_node = ROStoZMQBridge()
    zmq2ros_bridge_node = ZMQtoROSBridge()
    
    zmq_thread = threading.Thread(target=zmq2ros_bridge_node.zmq_to_ros)
    zmq_thread.start()
    
    try:
        rclpy.spin(ros2zmq_bridge_node)
    except KeyboardInterrupt:
        ros2zmq_bridge_node.get_logger().info("Shutting down bridge.")
    finally:
        ros2zmq_bridge_node.destroy()
        ros2zmq_bridge_node.get_logger().info("ZeroMQ context terminated.")
        zmq2ros_bridge_node.destroy()
        zmq_thread.join()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
