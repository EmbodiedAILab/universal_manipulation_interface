from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    eef_control_node = Node(
        package='umi_control',
        executable='arm_eef_controller',
        name='arm_eef_controller',
        parameters=[{'use_sim_time': True}],
        output="screen",
    )
    joint_control_node = Node(
        package='umi_control',
        executable='arm_joint_controller',
        name='arm_joint_controller',
        parameters=[{'use_sim_time': True}],
        output="screen",
    )
    nodes_to_start = [
        eef_control_node,
        joint_control_node,
    ]
    return LaunchDescription(nodes_to_start)
