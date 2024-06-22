import cv2
import os
import re

def extract_image_number(image_name):
    # 使用正则表达式提取图片名称中的序号
    match = re.search(r'rgb_(\d+)_\w+\.png', image_name)
    if match:
        return int(match.group(1))
    else:
        return -1  # 如果未能匹配到序号，返回一个负数作为默认值

def images_to_video(image_folder, output_video_file, fps):
    # 获取所有图片文件名并排序
    images = [img for img in os.listdir(image_folder) if img.endswith(".jpg") or img.endswith(".png")]
    images.sort(key=lambda x: extract_image_number(x))  # 根据提取的序号进行排序

    # 读取第一张图片以获取帧的宽度和高度
    first_image_path = os.path.join(image_folder, images[0])
    first_image = cv2.imread(first_image_path)
    height, width, layers = first_image.shape

    # 定义视频编写器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 指定视频编码器
    video = cv2.VideoWriter(output_video_file, fourcc, fps, (width, height))

    for image in images:
        image_path = os.path.join(image_folder, image)
        frame = cv2.imread(image_path)
        video.write(frame)

    video.release()

if __name__ == "__main__":
    image_folder = "/home/robot/Dev/ORB_SLAM3/dataset/MH01/mav0/cam0/data"  # 替换为你的图片文件夹路径
    output_video_file = "output_video.mp4"      # 替换为你想保存的视频文件名
    fps = 30                                    # 设置视频帧率
    images_to_video(image_folder, output_video_file, fps)