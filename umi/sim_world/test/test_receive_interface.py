import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        
        # 创建发布者
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.eef_pose_pub = self.create_publisher(Pose, "/eef_pose", 10)
        self.gripper_width_pub = self.create_publisher(Float64, "/gripper_width", 10)
        
        # 定时器，以固定频率发布消息
        timer_period = 1.0  # 每秒发布一次
        self.timer = self.create_timer(timer_period, self.publish_test_messages)

    def publish_test_messages(self):
        # 创建 JointState 消息
        joint_state_msg = JointState()
        joint_state_msg.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        joint_state_msg.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳

        # 发布 JointState 消息
        self.joint_state_pub.publish(joint_state_msg)
        self.get_logger().info("Published joint state message")

        # 创建 Pose 消息
        eef_pose_msg = Pose()
        eef_pose_msg.position.x = 0.5
        eef_pose_msg.position.y = 0.0
        eef_pose_msg.position.z = 0.7
        eef_pose_msg.orientation.x = 0.0
        eef_pose_msg.orientation.y = 0.0
        eef_pose_msg.orientation.z = 0.0
        eef_pose_msg.orientation.w = 1.0

        # 发布 eef_pose 消息
        self.eef_pose_pub.publish(eef_pose_msg)
        self.get_logger().info("Published eef pose message")

        # 创建 gripper_width 消息
        gripper_width_msg = Float64()
        gripper_width_msg.data = 0.05  # 夹爪宽度 5cm

        # 发布 gripper_width 消息
        self.gripper_width_pub.publish(gripper_width_msg)
        self.get_logger().info("Published gripper width message")

def main():
    rclpy.init()
    test_publisher = TestPublisher()
    
    try:
        rclpy.spin(test_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        test_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
