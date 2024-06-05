## 使用aruco记录夹爪的轨迹和视频

1. 使用ROS打开两个realsense相机，可以将末端相机的名称命名为`ego_camera`，外部的相机命名为`ext_camera`。可以使用`data_record/launch/start_two_realsense.launch`完成。当前未实现，同时需要注意修改每个相机的usb端口，防止不同实验中两个相机编号变化;
2. 使用使用`data_record/launch/start_aruco.launch`完成aruco的启动。注意安装相关的依赖，具体可以参考`start_usb_camera_usb_camera.launch`来实现。此外，如果想使用`start_usb_camera_usb_camera.launch`复现usb相机运行aruco的结果，注意在`~/.ros/camera_info`文件夹下面新建一个名称为`head_camera.yaml`的文件，然后填入以下信息，不然发布aruco信息会出错。
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

3. 在夹爪上黏贴二维码，注意二维码的编号和尺寸，其中[二维码生成网站](https://chev.me/arucogen/)的Dictionary选择Original Aruco，Marker ID和尺寸需要与上面的launch文件对应；

4. 运行`data_record`下面的`start_record.launch`，确保aruco能被检测到。然后按下s开始记录，按下e结束记录。如此循环，就可以生成多条数据。数据的地址存放在`record_data/data`下面；
