import click
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.widgets import Slider, Button
import hydra
from omegaconf import OmegaConf


matplotlib.use('TkAgg')

def load_dataset(dataset_path):
    OmegaConf.register_new_resolver("eval", eval, replace=True)

    with hydra.initialize('../diffusion_policy/config'):
        cfg = hydra.compose('train_diffusion_unet_timm_umi_workspace')
        OmegaConf.resolve(cfg)
        cfg.task.dataset.dataset_path = dataset_path
        dataset = hydra.utils.instantiate(cfg.task.dataset)

    return dataset.replay_buffer
class DatasetVisualizer:
    def __init__(self, replay_buffer):
        # 创建绘图窗口
        self.fig = plt.figure(figsize=(18, 10))
        self.ax1 = self.fig.add_subplot(131, projection='3d')
        self.ax2 = self.fig.add_subplot(132)
        self.ax2.set_title('camera_0')
        self.ax2.axis('off')
        self.ax3 = self.fig.add_subplot(132)
        self.ax3.set_title('camera_1')
        self.ax3.axis('off')

        # 初始化空的轨迹线
        # self.line, = self.ax.plot([], [], [], label='3D trajectory', lw=0.1)
        self.points, = self.ax1.plot([], [], [], 'ro',linestyle='None', markersize=10, label='Points')

        # 设置轴标签和范围
        self.ax1.set_xlabel('X axis')
        self.ax1.set_ylabel('Y axis')
        self.ax1.set_zlabel('Z axis')
        self.ax1.set_title('3D Trajectory')

        #创建播放时间显示文本
        self.time_text = self.ax1.text2D(0.05, 0.95, '', transform=self.ax1.transAxes)

        # 创建滑块
        self.ax_slider = plt.axes([0.1, 0.02, 0.65, 0.03], facecolor='lightgoldenrodyellow')

        # 创建播放和暂停按钮
        self.ax_play = plt.axes([0.8, 0.025, 0.05, 0.04])
        self.btn_play = Button(self.ax_play, 'Play', color='lightgoldenrodyellow', hovercolor='0.975')

        self.ax_pause = plt.axes([0.85, 0.025, 0.05, 0.04])
        self.btn_pause = Button(self.ax_pause, 'Pause', color='lightgoldenrodyellow', hovercolor='0.975')

        self.ax_prev = plt.axes([0.9, 0.025, 0.05, 0.04])
        self.btn_prev = Button(self.ax_prev, 'Prev', color='lightgoldenrodyellow', hovercolor='0.975')

        self.ax_next = plt.axes([0.95, 0.025, 0.05, 0.04])
        self.btn_next = Button(self.ax_next, 'Next', color='lightgoldenrodyellow', hovercolor='0.975')

        # 连接按钮点击事件
        self.btn_play.on_clicked(self.play)
        self.btn_pause.on_clicked(self.pause)
        self.btn_prev.on_clicked(self.prev)
        self.btn_next.on_clicked(self.next)

        self.replay_buffer = replay_buffer

    def update(self, val):
        num = int(self.slider.val)
        # self.line.set_data(self.x[num], self.y[num])
        # self.line.set_3d_properties(self.z[num])
        self.points.set_data(self.x[num-1], self.y[num-1])
        self.points.set_3d_properties(self.z[num-1])
        self.camera0_dis.set_data(self.camera0[num - 1])
        self.camera1_dis.set_data(self.camera1[num - 1])

        current_time = self.timestamps[num-1]/30
        self.time_text.set_text(f'Time: {current_time:.2f} s')
        self.fig.canvas.draw_idle()

    def play(self, event):
        self.playing = True
        while self.playing and self.slider.val < len(self.x) - 1:
            self.slider.set_val(self.slider.val + 3)
            plt.pause(1/30)

    def pause(self, event):
        self.playing = False

    def prev(self, event):
        if self.n_episode > 0:
            self.n_episode -= 1
            self.load('')
            self.fig.canvas.draw_idle()

    def next(self, event):
        if self.n_episode < self.replay_buffer.n_episodes:
            self.n_episode += 1
            self.load('')
            self.fig.canvas.draw_idle()

    def load(self, episode_id):
        self.n_episode = episode_id
        pos = self.replay_buffer.get_episode(self.n_episode)['robot0_eef_pos']

        self.x = pos[:,0]
        self.y = pos[:,1]
        self.z = pos[:,2]
        # self.timestamps = replay_buffer['timestamp']
        self.timestamps = np.arange(0,len(self.x), 1)
        self.camera0 = self.replay_buffer.get_episode(self.n_episode)['camera_0']
        self.camera1 = self.replay_buffer.get_episode(self.n_episode)['camera_0']


        self.ax1.set_xlim(min(self.x), max(self.x))
        self.ax1.set_ylim(min(self.y), max(self.y))
        self.ax1.set_zlim(min(self.y), max(self.z))

        self.slider = Slider(self.ax_slider, 'Time', 0, len(self.x)-1, valinit=0, valstep=1)
        # 将滑块与更新函数连接
        self.slider.on_changed(self.update)
        # 初始化图形
        #self.update(self.x,self.y,self.z)

        self.ax1.legend()

        self.camera0_dis = self.ax2.imshow(self.camera0[0])
        self.camera1_dis = self.ax3.imshow(self.camera1[1])

        print(f"pos: {pos.shape}, camera0: {self.camera0.shape}")


    def show(self, episode_id):
        self.load(episode_id)
        plt.show()

@click.command()
@click.option('--path', default='../../../push_switch7')
def main(path):
    replay_buffer = load_dataset(path)
    animator = DatasetVisualizer(replay_buffer)
    animator.show(0)

if __name__ == '__main__':
    main()

