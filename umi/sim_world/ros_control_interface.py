import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import transformations as tft


class ROSControlInterface(Node):
    def __init__(self):
        super().__init__('ros_control_interface')
        
        self.servoL_pub = self.create_publisher(PoseStamped, '/servoL_cmd', 10)
        self.moveJ_pub = self.create_publisher(JointState, '/moveJ_cmd', 10)
        self.gripper_state_pub = self.create_publisher(JointState, '/gripper_cmd', 10)

    def servoL(self, pose):
        """
        将 [x, y, z, rx, ry, rz] 的位姿作为输入参数，并发布到 /servoL_cmd 主题上
        :param pose: 位姿数据 [x, y, z, rx, ry, rz]
        """
        if len(pose) != 6:
            self.get_logger().error("Pose must have 6 elements: [x, y, z, rx, ry, rz]")
            return
        
        # 创建 PoseStamped 消息
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "tool_link"

        # 填充位置
        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = pose[2]

        # 填充旋转四元数
        euler = [pose[3], pose[4], pose[5]]
        quaternion = tft.quaternion_from_euler(*euler)
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]

        # 发布消息
        self.servoL_pub.publish(pose_stamped)
        self.get_logger().info(f"Published Pose: {pose_stamped}")
    
    def moveJ(self, joint_positions):
        if not isinstance(joint_positions, list) or not all(isinstance(pos, (int, float)) for pos in joint_positions):
            self.get_logger().error("Joint positions must be a list of numbers")
            return

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = "base_link"  # 修改 frame_id 根据需要
        joint_state_msg.name = [f'joint_{i}' for i in range(len(joint_positions))]  # 根据需要修改关节名称
        joint_state_msg.position = joint_positions
        joint_state_msg.velocity = []  # 速度为空
        joint_state_msg.effort = []    # 力矩为空

        self.moveJ_pub.publish(joint_state_msg)
        self.get_logger().info(f"Published JointState: {joint_positions}")
        
    def setGripperTargetPos(self, target_pos):
        joint_positions = [-target_pos/2, -target_pos/2]
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = "tool_link"  # 修改 frame_id 根据需要
        joint_state_msg.name = ["gripper"]
        joint_state_msg.position = joint_positions
        joint_state_msg.velocity = []  # 速度为空
        joint_state_msg.effort = []    # 力矩为空
        
        self.gripper_state_pub.publish(joint_state_msg)
        self.get_logger().info(f"Published Gripper JointState: {joint_positions}")

def main(args=None):
    rclpy.init(args=args)
    control_interface = ROSControlInterface()
    
    # 示例 pose 数据 [x, y, z, rx, ry, rz]
    example_pose = [1.0, 0.0, 0.5, 0.0, 0.0, 0.0]
        
    try:
        # rate = control_interface.create_rate(1)  # 设置循环频率为1Hz
        while rclpy.ok():
            control_interface.servoL(example_pose)
            control_interface.get_logger().info("servoL called.")  # 调试信息
        
    except Exception as e:
        control_interface.get_logger().error(f"Exception occurred: {str(e)}")
    except KeyboardInterrupt:
        control_interface.get_logger().info("Keyboard interrupt received.")
    finally:
        control_interface.destroy_node()
        rclpy.shutdown()
        control_interface.get_logger().info("Node destroyed and shutdown complete.")
    

if __name__ == "__main__":
    main()

