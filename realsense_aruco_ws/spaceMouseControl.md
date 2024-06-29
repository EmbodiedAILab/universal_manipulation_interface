## spaceMouse控制机械臂运动

### 安装必要的依赖

1. 安装spaceMouse的驱动

    ```
    sudo apt install spacenavd
    sudo apt install ros-noetic-spacenav-node
    ```

2. 安装moveit相关的包
    ```
    sudo apt install ros-noetic-moveit*
    sudo apt install ros-noetic-trac-ik*
    ```

3. 根据提示安装其他的依赖包

### 运行

1. 启动moveit仿真环境
   
   ```
    roslaunch umi_moveit_config demo.launch
   ```

2. 启动末端运动控制
   ```
    rosrun umi_control umi_control_from_spacemouse
   ```
    可以看见机器人末端有规律的运动

### 修改事项

`umi_control_from_spacemouse.cpp`里面，把`isTest`变量设置为`false`。并且在`spaceMouseCallback(const sensor_msgs::JoyConstPtr& joyPtr)`函数里面，将spaceMouse的输出与末端和夹爪的运动**合理的**映射起来。

此外，当前夹爪控制比较粗暴，只有开合，没有中间状态，后期根据情况可以简单些一个线性插值控制器。
