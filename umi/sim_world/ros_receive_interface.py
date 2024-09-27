import rospy
import tf
import threading
from sensor_msgs.msg import JointState
import numpy as np

class ROSReceiveInterface:
    def __init__(self):
        rospy.init_node('hand_link_transform_listener', anonymous=True)
        self.listener = tf.TransformListener()
        self.lock = threading.Lock()
        self.tf_data = None
        self.arm_joint_positions = []
        self.gripper_joint_positions = []
        
        # Subscribe to /joint_states topic
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

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
            with self.lock:
                # Directly lookup the transform from hand_link to base_link_inertia
                (trans, rot) = self.listener.lookupTransform('/base_link_inertia', '/hand_link', rospy.Time(0))

                # Convert quaternion (rot) to axis-angle (rx, ry, rz)
                axis_angle = self.quaternion_to_axis_angle(rot)

                # Return the pose in the format [x, y, z, rx, ry, rz]
                return [trans[0], trans[1], trans[2], axis_angle[0], axis_angle[1], axis_angle[2]]
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", e)
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
    ros_client = ROSReceiveInterface()
    while not rospy.is_shutdown():
        rospy.sleep(1)  # Give time for tf listener and joint_states to buffer some data
        print("Actual TCP Pose:", ros_client.getActualTCPPose())
        print("Actual Q:", ros_client.getActualQ())
        print("Gripper:", ros_client.getGripperCurrentPos())