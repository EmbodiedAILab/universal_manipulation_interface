import rospy
import csv
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans
import time

def read_csv(file_path):
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file)
        data = list(reader)
    return data

def publish_poses(data, rate_hz=10):
    rospy.init_node('csv_pose_publisher', anonymous=True)
    pose_pub = rospy.Publisher('/servoL_cmd', PoseStamped, queue_size=10)
    rate = rospy.Rate(rate_hz)
    seq = 0

    while not rospy.is_shutdown():
        for row in data:
            # 解析CSV中的位姿数据
            pose = PoseStamped()
            pose.header.seq = seq
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "tool_link" 

            pose.pose.position.x = float(row['x'])
            pose.pose.position.y = float(row['y'])
            pose.pose.position.z = float(row['z'])
            pose.pose.orientation.x = float(row['q_x'])
            pose.pose.orientation.y = float(row['q_y'])
            pose.pose.orientation.z = float(row['q_z'])
            pose.pose.orientation.w = float(row['q_w'])

            pose_pub.publish(pose)
            seq += 1
            rate.sleep()

if __name__ == '__main__':
    try:
        file_path = "/home/robot/桌面/pose_test.csv" 
        data = read_csv(file_path)
        if not data:
            rospy.logerr("No data found in the CSV file.")
            exit(1)
        publish_poses(data)
    except rospy.ROSInterruptException:
        pass
