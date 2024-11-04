import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        
        # 创建 TransformBroadcaster 对象
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 初始化计时器
        self.start_time = time.time()

        # 定时器，设置发布频率
        self.timer = self.create_timer(0.1, self.publish_transform)  # 每0.1秒发布一次TF

    def publish_transform(self):
        # 计算时间间隔，用于动态变化
        elapsed = time.time() - self.start_time
        
        # 创建TransformStamped消息
        t = TransformStamped()
        
        # 设置父子坐标系
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base'
        t.child_frame_id = 'hand_link'
        
        # 设置平移和旋转。这里我们让平移和旋转随着时间变化
        t.transform.translation.x = 0.5 * math.sin(elapsed)  # x方向平移随时间变化
        t.transform.translation.y = 0.5 * math.cos(elapsed)  # y方向平移随时间变化
        t.transform.translation.z = 0.2                     # z方向保持不变

        # 设置四元数表示的旋转，这里让绕z轴的旋转随时间变化
        angle = math.radians(elapsed * 10)  # 让旋转角度随时间变化
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(angle / 2)
        t.transform.rotation.w = math.cos(angle / 2)

        # 发布 TF
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Publishing TF: base -> hand_link at {elapsed:.2f}s")

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
