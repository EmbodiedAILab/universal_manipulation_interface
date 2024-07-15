#!/usr/bin/env python

import rospy
import pandas as pd
from geometry_msgs.msg import TransformStamped
import tf2_ros
import sys

def publish_poses_from_csv(csv_file):
    # 读取CSV文件
    df = pd.read_csv(csv_file)

    # 初始化ROS节点
    rospy.init_node('pose_publisher', anonymous=True)
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(30) # 10 Hz

    for index, row in df.iterrows():
        if rospy.is_shutdown():
            break
        
        # 创建TransformStamped消息
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "sim_end_effector"
        
        # 设置平移
        t.transform.translation.x = row['x']
        t.transform.translation.y = row['y']
        t.transform.translation.z = row['z']
        
        # 设置旋转（四元数）
        t.transform.rotation.x = row['q_x']
        t.transform.rotation.y = row['q_y']
        t.transform.rotation.z = row['q_z']
        t.transform.rotation.w = row['q_w']
        
        # 发布TF
        br.sendTransform(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        # 从命令行参数获取CSV文件路径
        if len(sys.argv) < 2:
            rospy.logerr("Usage: rosrun pose_publisher publish_poses.py <csv_file_path>")
            sys.exit(1)
        csv_file_path = sys.argv[1]
        
        publish_poses_from_csv(csv_file_path)
    except rospy.ROSInterruptException:
        pass