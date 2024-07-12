# recodring_processor.py 脚本引导
recording_processor.py脚本用于对ME仿真环境产生的UMI数据（图片集、夹爪宽度、轨迹）进行后处理。

## 功能

1. 监视 `recording_folder` 目录中的新产生的子目录`umi_{date}_{time}` 。
2. 当子目录包含 `true` 或 `false` 文件时进行处理。
3. 从 `pictures` 子目录中的图像生成视频。
4. 调整 CSV 文件中的时间戳，将夹爪宽度和轨迹数据的时间戳和视频帧对齐。
5. 将处理后的文件移动到 `succeded_data` 或 `failed_data` 子目录。
6. 删除处理后的 `umi_` 子目录。
7. 计数并报告成功和失败的操作次数。

## 依赖项

脚本需要以下 Python 库：
- `opencv-python`
- `watchdog`

可以使用 pip 安装它们：
```bash
pip install opencv-python watchdog
```

## 处理前后目录变化
处理前，ME录制文件目录：
```
recording_folder/
├── umi_21-07-11_12-34-56/
│   ├── pictures/
│   │   ├── rgb_0_2021-07-11_12-34-56.png
│   │   ├── rgb_1_2021-07-11_12-34-57.png
│   │   └── ...
│   ├── jaw_width.csv
│   ├── position.csv
│   ├── raw_video.mp4
│   ├── true / false
```

处理后目录（根据后续训练前处理要求的格式，将数据统一放到一个目录）：
```
recording_folder/
├── succeded_data/
│   ├── gripper_21-07-11_12-34-56.csv
│   ├── pose_21-07-11_12-34-56.csv
│   └── video_21-07-11_12-34-56.mp4
|   └── ...
├── failed_data/
│   ├── gripper_21-07-11_12-34-56.csv
│   ├── pose_21-07-11_12-34-56.csv
│   └── video_21-07-11_12-34-56.mp4
|   └── ...
```
## 启动
设置环境变量
```
# 设置处理数据的目录
export RECORDING_FOLDER=/path/to/your/recording
```
启动

```
python3 your_script.py
```
