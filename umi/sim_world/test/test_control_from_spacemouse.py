import rospy
import tf
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from threading import Lock, Thread
import tf.transformations as tft
import queue

position_step = 0.01
rotation_step = 0.01

class ROSControlInterface:
    def __init__(self, publish_rate=10):
        rospy.init_node('ros_control_interface', anonymous=True)
        self.lock = Lock()
        self.servoL_pub = rospy.Publisher('/servoL_cmd', PoseStamped, queue_size=10)
        self.latest_pose = None
        self.rate = rospy.Rate(publish_rate) 

        # 启动异步处理线程
        Thread(target=self.publish_latest_pose, daemon=True).start()

    def publish_latest_pose(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.latest_pose is not None:
                    self._publish_pose(self.latest_pose)
            self.rate.sleep()
            
    def _publish_pose(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "tool_link"
        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = pose[2]

        euler = [pose[3], pose[4], pose[5]]
        quaternion = tft.quaternion_from_euler(*euler)
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]

        self.servoL_pub.publish(pose_stamped)
        rospy.loginfo("Published Pose: %s", pose_stamped)

    def servoL(self, pose):
        with self.lock:
            self.latest_pose = pose 
            

ros_control_interface = ROSControlInterface(publish_rate=30)

def get_current_pose(listener, target_frame, source_frame):
    try:
        listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        return [trans[0], trans[1], trans[2], euler[0], euler[1], euler[2]]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("TF lookup failed.")
        return None

def joy_callback(msg, listener):
    current_pose = get_current_pose(listener, 'world', 'tool_link')
    if current_pose is None:
        return

    current_pose[0] -= msg.axes[0] * position_step * 0.7
    current_pose[1] -= msg.axes[1] * position_step * 0.7
    current_pose[2] -= msg.axes[2] * position_step * 0.7
    current_pose[3] += msg.axes[3] * rotation_step
    current_pose[4] += msg.axes[4] * rotation_step
    current_pose[5] += msg.axes[5] * rotation_step

    ros_control_interface.servoL(current_pose)

def main():
    if not rospy.core.is_initialized():
        rospy.init_node('joy_to_pose_servoL', anonymous=True)

    listener = tf.TransformListener()
    rospy.Subscriber("/spacenav/joy", Joy, joy_callback, listener)
    rospy.loginfo("Started joy_to_pose_servoL node, waiting for /spacenav/joy messages...")
    rospy.spin()

if __name__ == '__main__':
    main()
