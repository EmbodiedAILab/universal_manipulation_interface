import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros
import tf_transformations
import threading
import numpy as np
from geometry_msgs.msg import TransformStamped

class ROSReceiveInterface(Node):
    def __init__(self):
        super().__init__('hand_link_transform_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.lock = threading.Lock()
        self.arm_joint_positions = []
        self.gripper_joint_positions = []
        
        # Subscribe to /joint_states topic
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

    def joint_states_callback(self, msg):
        with self.lock:
            self.arm_joint_positions = list(msg.position[:6])
            self.gripper_joint_positions = list(msg.position[-2:])

    def quaternion_to_axis_angle(self, quat):
        """
        Converts a quaternion to axis-angle representation.
        quat: [x, y, z, w] - input quaternion
        returns: [rx, ry, rz] - axis-angle representation
        """
        # Unpack the quaternion
        qx, qy, qz, qw = quat

        # Compute the angle
        angle = 2 * np.arccos(qw)

        # Compute the normalization factor
        sin_theta_over_two = np.sqrt(1 - qw * qw)

        # Avoid division by zero
        if sin_theta_over_two < 1e-6:
            return [0.0, 0.0, 0.0]  # If the angle is zero, any axis will work.

        # Compute the axis
        rx = qx / sin_theta_over_two
        ry = qy / sin_theta_over_two
        rz = qz / sin_theta_over_two

        # Scale the axis by the angle to get the axis-angle representation
        return [rx * angle, ry * angle, rz * angle]

    # ================= Arm Status API ===================    
    def getActualTCPPose(self):
        try:
            # Directly lookup the transform from hand_link to base_link_inertia
            trans = self.tf_buffer.lookup_transform('base', 'hand_link', rclpy.time.Time())
            # Convert quaternion (rot) to axis-angle (rx, ry, rz)
            axis_angle = self.quaternion_to_axis_angle([trans.transform.rotation.x,
                                                         trans.transform.rotation.y,
                                                         trans.transform.rotation.z,
                                                         trans.transform.rotation.w])

            # Return the pose in the format [x, y, z, rx, ry, rz]
            return [trans.transform.translation.x, 
                    trans.transform.translation.y, 
                    trans.transform.translation.z, 
                    axis_angle[0], 
                    axis_angle[1], 
                    axis_angle[2]]
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform error: {e}")
            return None
    
    def getActualTCPSpeed(self):
        return

    def getActualQ(self):
        with self.lock:
            # Return the joint positions in a list format matching rtde_receive's getActualQ output
            return self.arm_joint_positions if self.arm_joint_positions else []
    
    def getActualQd(self):
        return
    
    def getTargetTCPPose(self):
        return  

    def getTargetTCPSpeed(self):
        return

    def getTargetQ(self):
        return

    def getTargetQd(self):
        return
    
    # ================= Gripper Status API ===================    
    def getGripperCurrentPos(self):
        with self.lock:
            position = 0.08
            if self.gripper_joint_positions:
                position = abs(self.gripper_joint_positions[0] + self.gripper_joint_positions[1])
            return position
    
if __name__ == "__main__":
    rclpy.init()
    ros_client = ROSReceiveInterface()
    try:
        while rclpy.ok():
            rclpy.spin_once(ros_client)
            print("Actual TCP Pose:", ros_client.getActualTCPPose())
            print("Actual Q:", ros_client.getActualQ())
            print("Gripper:", ros_client.getGripperCurrentPos())
            # ros_client.get_clock().sleep_for(1)  # Give time for tf listener and joint_states to buffer some data
    except KeyboardInterrupt:
        pass
    finally:
        ros_client.destroy_node()
        rclpy.shutdown()
