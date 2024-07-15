pose_publisher ROS节点用于读取UMI采集的轨迹，从pose.csv顺序读取轨迹并播放，发布为相对sim_end_effector相对于world坐标系的tf topic

### 使用方式
1. 编译
   工作空间下使用 `catkin_make` 编译
2. 运行节点
   ```
   roscore (已启动rosmaster则忽略)
   rosrun pose_publisher publish_poses.py /path/to/pose_{xxx}.csv
   ```
3. 打开rviz 选中 tf 中的 world 与 sim_end_effector 观察运动趋势