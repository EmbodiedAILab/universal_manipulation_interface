## 使用aruco记录夹爪的轨迹和视频

### 启动两个realsense

1. 安装realsense相关的驱动

    ```
    sudo apt install ros-noetic-realsense*
    ```

2. 获取相机的序列号
将两个相机分别插入电脑的usb口，然后运行以下命令，命令行会显示相机的序列号，比如`728312070349`这样的一串数字
    ```
    roslaunch realsense2_camera rs_camera.launch
    ```
3. 将两个序列号填入`data_record/launch/start_two_realsense.launch`中的`serial_no_camera1`和`serial_no_camera2`。在本实验中，`serial_no_camera1`对应的是夹爪上的相机，`serial_no_camera2`对应的是第三视角的相机

4. 运行以下命令，查看是否启动正常，注意`soruce`相关的环境变量

    ```
    roslaunch record_data start_two_realsense.launch
    ```

    查看两个相机的图片是否正常
    ```
    rosrun image_view image_view image:=/ego_camera/color/image_raw
    rosrun image_view image_view image:=/ext_camera/color/image_raw
    ```

### 测试外置相机识别二维码

1. 确保两个相机正常启动，且外置相机的topic为`/ext_camera/color/image_raw`;
2. 在夹爪上黏贴二维码，注意二维码的编号和尺寸，其中[二维码生成网站](https://chev.me/arucogen/)的Dictionary选择Original Aruco，Marker ID和尺寸需要与launch文件对应；
3. 在命令行启动如下命令:
    ```
    roslaunch record_data start_aruco.launch
    ```
4. 查看检测结果
    ```
    rosrun image_view image_view image:=/aruco_marker_publisher/result
    ```
如果有没有二维码检测出来，注意检查launch文件中的字段是否正确；

5. 其他：`start_usb_camera_usb_camera.launch`是使用usb相机识别aruco的示例代码。注意如果想使用`start_usb_camera_usb_camera.launch`复现usb相机运行aruco的结果，注意在`~/.ros/camera_info`文件夹下面新建一个名称为`head_camera.yaml`的文件，然后填入以下信息，不然发布aruco信息会出错。
    ```
    image_width: 640
    image_height: 480
    camera_name: head_camera
    camera_matrix:
    rows: 3
    cols: 3
    data: [438.783367, 0.000000, 305.593336, 0.000000, 437.302876, 243.738352, 0.000000, 0.000000, 1.000000]
    distortion_model: plumb_bob
    distortion_coefficients:
    rows: 1
    cols: 5
    data: [-0.361976, 0.110510, 0.001014, 0.000505, 0.000000]
    rectification_matrix:
    rows: 3
    cols: 3
    data: [0.999978, 0.002789, -0.006046, -0.002816, 0.999986, -0.004401, 0.006034, 0.004417, 0.999972]
    projection_matrix:
    rows: 3
    cols: 4
    data: [393.653800, 0.000000, 322.797939, 0.000000, 0.000000, 393.653800, 241.090902, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
    ```

### 记录数据

1. 关闭上述所有ros节点，然后运行下列命令，同时启动两个realsense和aruco检测程序
    ```
    roslaunch record_data start_realsense_and_aruco.launch
    ```

2. 运行下列程序进行数据记录
   ```
    roslaunch record_data start_record.launch
   ```
    在记录过程中，确保aruco能被检测到。然后按下s开始记录，按下e结束记录。如此循环，就可以生成多条数据。数据的地址存放在`record_data/data`下面；
