import zmq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import pickle

# UR5e 的官方关节名称顺序
UR5E_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

class ROSZMQBridge(Node):
    def __init__(self, zmq_host="localhost", zmq_port=5555):
        super().__init__('ros_zmq_bridge')
        
        # 初始化 ZeroMQ 上下文和套接字
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{zmq_host}:{zmq_port}")

        # 订阅 /joint_states 主题
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        # 订阅 /eef_pose 主题
        self.create_subscription(Pose, "/eef_pose", self.eef_pose_callback, 10)
        # 订阅 /gripper_width 主题
        self.create_subscription(Float64, "/gripper_width", self.gripper_width_callback, 10)

        # 保存最近的 eef_pose 和 gripper_width
        self.latest_eef_pose = None
        self.latest_gripper_width = None

    def joint_states_callback(self, msg):
        # 将 JointState 消息转换为字典，并按照 UR5e 官方关节顺序对关节位置进行排序
        joint_positions = [0.0] * 6  # 初始化一个长度为 6 的列表

        # 遍历 msg.name 和 msg.position，确保按照官方顺序填充
        for i, joint_name in enumerate(msg.name):
            if joint_name in UR5E_JOINTS:
                index = UR5E_JOINTS.index(joint_name)  # 获取该关节名称在官方顺序中的位置
                joint_positions[index] = msg.position[i]

        # 将数据打包
        joint_state_data = {
            'positions': joint_positions,
            'timestamp': self.get_clock().now().nanoseconds
        }

        # 发送 joint_states_cmd 主题
        serialized_data = pickle.dumps(joint_state_data)
        self.socket.send_multipart([b'joint_states_cmd', serialized_data])
        self.get_logger().info(f"Sent joint_states_cmd: {joint_state_data}")

    def eef_pose_callback(self, msg):
        # 将 eef_pose 数据打包
        eef_pose_data = {
            'position': {
                'x': msg.position.x,
                'y': msg.position.y,
                'z': msg.position.z
            },
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'timestamp': self.get_clock().now().nanoseconds
        }

        # 发送 eef_pose_cmd 主题
        serialized_data = pickle.dumps(eef_pose_data)
        self.socket.send_multipart([b'eef_pose_cmd', serialized_data])
        self.get_logger().info(f"Sent eef_pose_cmd: {eef_pose_data}")

    def gripper_width_callback(self, msg):
        # 将 gripper_width 数据打包
        gripper_width_data = {
            'width': msg.data,
            'timestamp': self.get_clock().now().nanoseconds
        }

        # 发送 gripper_width_cmd 主题
        serialized_data = pickle.dumps(gripper_width_data)
        self.socket.send_multipart([b'gripper_width_cmd', serialized_data])
        self.get_logger().info(f"Sent gripper_width_cmd: {gripper_width_data}")

    def destroy(self):
        self.socket.close()
        self.zmq_context.term()

if __name__ == "__main__":
    rclpy.init()
    bridge_node = ROSZMQBridge()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy()
        rclpy.shutdown()
