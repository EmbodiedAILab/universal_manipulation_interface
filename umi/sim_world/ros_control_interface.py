import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf.transformations as tft

class ROSControlInterface:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('ros_control_interface', anonymous=True)
        
        # 创建发布器，发布到 /arm_servo_cmd 主题
        self.servoL_pub = rospy.Publisher('/servoL_cmd', PoseStamped, queue_size=10)
        self.moveJ_pub = rospy.Publisher('/moveJ_cmd', JointState, queue_size=10)
        self.gripper_state_pub = rospy.Publisher('/gripper_cmd', JointState, queue_size=10)
        

    def servoL(self, pose):
        """
        将 [x, y, z, rx, ry, rz] 的位姿作为输入参数，并发布到 /arm_servo_cmd 主题上
        :param pose: 位姿数据 [x, y, z, rx, ry, rz]
        """
        if len(pose) != 6:
            rospy.logerr("Pose must have 6 elements: [x, y, z, rx, ry, rz]")
            return
        
        # 创建 PoseStamped 消息
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
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
        rospy.loginfo("Published Pose: %s", pose_stamped)
    
    def moveJ(self, joint_positions):
        if not isinstance(joint_positions, list) or not all(isinstance(pos, (int, float)) for pos in joint_positions):
            rospy.logerr("Joint positions must be a list of numbers")
            return

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.header.frame_id = "base_link"  # Modify frame_id if needed
        joint_state_msg.name = [f'joint_{i}' for i in range(len(joint_positions))]  # Modify joint names if needed
        joint_state_msg.position = joint_positions
        joint_state_msg.velocity = []  # Empty list for velocity, as not provided
        joint_state_msg.effort = []    # Empty list for effort, as not provided

        self.moveJ_pub.publish(joint_state_msg)
        rospy.loginfo("Published JointState: %s", joint_positions)
        
    def setGripperTargetPos(self, target_pos):
        joint_positions = [-target_pos/2, -target_pos/2]
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.header.frame_id = "tool_link"  # Modify frame_id if needed
        joint_state_msg.name = ["gripper"]
        joint_state_msg.position = joint_positions
        joint_state_msg.velocity = []  # Empty list for velocity, as not provided
        joint_state_msg.effort = []    # Empty list for effort, as not provided
        
        self.gripper_state_pub.publish(joint_state_msg)
        rospy.loginfo("Published Gripper JointState: %s", joint_positions)

if __name__ == "__main__":
    # 创建 ROSControlInterface 对象
    control_interface = ROSControlInterface()
    
    # 示例 pose 数据 [x, y, z, rx, ry, rz]
    example_pose = [1.0, 0.0, 0.5, 0.0, 0.0, 0.0]
    example_joint = [0, 0, 0, 0, 0, 0]
    example_gripper_pos = 0.08
    
    while not rospy.is_shutdown():
        control_interface.servoL(example_pose)
        # control_interface.moveJ(example_joint)
        # control_interface.setGripperTargetPos(example_gripper_pos)