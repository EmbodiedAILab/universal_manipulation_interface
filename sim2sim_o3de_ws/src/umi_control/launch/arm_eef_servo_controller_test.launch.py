from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    eef_control_node_test = Node(
        package='umi_control',
        executable='arm_eef_controller_test',
        name='arm_eef_controller_test',
        parameters=[{'use_sim_time': True}],
        output="screen",
    )
    nodes_to_start = [
        eef_control_node_test,
    ]
    return LaunchDescription(nodes_to_start)
