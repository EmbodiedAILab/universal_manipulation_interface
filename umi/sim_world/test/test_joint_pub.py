import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_states_publisher')
        
        # 创建JointState发布器
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # 定时器，设置发布频率
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 每0.1秒发布一次JointState
        
        # 初始化计时器
        self.start_time = time.time()

        # 定义UR5的6个关节和两个夹爪的关节名称
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
            'left_finger_joint',
            'right_finger_joint'
        ]
    
    def publish_joint_states(self):
        # 计算时间间隔，用于动态变化
        elapsed = time.time() - self.start_time
        
        # 创建JointState消息
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names

        # 设置UR5的6个关节的角度，角度随时间变化
        joint_positions = [
            math.sin(elapsed),          # shoulder_pan_joint
            math.cos(elapsed),          # shoulder_lift_joint
            math.sin(elapsed) * 0.5,    # elbow_joint
            math.cos(elapsed) * 0.5,    # wrist_1_joint
            math.sin(elapsed) * 0.3,    # wrist_2_joint
            math.cos(elapsed) * 0.3     # wrist_3_joint
        ]

        # 设置夹爪关节的位置，范围在 0 到 -0.04
        gripper_position = -0.02 * (1 + math.sin(elapsed))  # 夹爪的范围在 0 到 -0.04之间变化
        joint_positions.append(gripper_position)  # left_finger_joint
        joint_positions.append(gripper_position)  # right_finger_joint

        joint_state_msg.position = joint_positions

        # 发布JointState消息
        self.joint_state_pub.publish(joint_state_msg)
        self.get_logger().info(f"Publishing joint states: {joint_positions}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
