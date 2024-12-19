from setuptools import setup
from glob import glob

package_name = 'zmq_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='171160276@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'umi_bridge = zmq_bridge.umi_bridge:main',
            'r1_bridge = zmq_bridge.r1_bridge:main',
            'camera_bridge = zmq_bridge.camera_bridge:main',
            'states_bridge = zmq_bridge.states_bridge:main',
            'arm_control_bridge = zmq_bridge.arm_control_bridge:main',
            'gripper_control_bridge = zmq_bridge.gripper_control_bridge:main',
        ],
    },
)
