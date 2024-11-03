from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("umi").to_moveit_configs()

    eef_control_node = Node(
        package='umi_control',
        executable='arm_eef_controller',
        name='arm_eef_controller',
        parameters=[{'use_sim_time': True},
                    moveit_config.robot_description_kinematics],
        output="screen",
    )
    joint_control_node = Node(
        package='umi_control',
        executable='arm_joint_controller',
        name='arm_joint_controller',
        parameters=[{'use_sim_time': True}],
        output="screen",
    )
    eef_pose_gripper_width_pub_node = Node(
            package='umi_control',
            executable='eef_pose_gripper_width_pub',
            name='eef_pose_gripper_width_pub',
            parameters=[{'use_sim_time': True}],
            output="screen",
        )

    nodes_to_start = [
        eef_control_node,
        joint_control_node,
        eef_pose_gripper_width_pub_node,
    ]
    return LaunchDescription(nodes_to_start)
