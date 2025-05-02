from setuptools import setup
import os
from glob import glob

package_name = 'ros_awsiot_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'test'), glob('test/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tadashi imokawa',
    maintainer_email='tadashi.imokawa@whill.inc',
    description='Agent for connecting AWS IoT and ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'named_shadow = ros_awsiot_agent.named_shadow:main',
            'talker = ros_awsiot_agent.test_talker:main',
            'listener = ros_awsiot_agent.test_listener:main',
            'ros2mqtt = ros_awsiot_agent.ros2mqtt:main',
            'mqtt2ros = ros_awsiot_agent.mqtt2ros:main',
        ],
    },
    tests_require=['pytest'],
    test_suite='pytest',
)
