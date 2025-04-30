from launch import LaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import os.path
from uuid import uuid4
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, TextSubstitution


def generate_launch_description():

    ros2mqtt_node = Node(
        package='ros_awsiot_agent',
        executable='ros2mqtt',
        name='ros2mqtt',
        output='screen',
        parameters=[
            {'topic_from': '/input'},
            {'topic_to': EnvironmentVariable('AWSIOT_MQTT_TOPIC')},
            {'cert': PathJoinSubstitution(
                [EnvironmentVariable('AWSIOT_CERT_DIR'), 'certificate.pem.crt'])},
            {'key': PathJoinSubstitution(
                [EnvironmentVariable('AWSIOT_CERT_DIR'), 'private.pem.key'])},
            {'root_ca': os.path.expanduser(
                '~/.aws/cert_root/AmazonRootCA1.pem')},
            {'endpoint': EnvironmentVariable('AWSIOT_ENDPOINT')},
            {'client_id': [EnvironmentVariable(
                'AWSIOT_THING_NAME'), TextSubstitution(text='-ros2mqtt')]},
            {'signing_region': 'ap-northeast-1'},
            {'use_websocket': False}
        ]
    )

    mqtt2ros_node = Node(
        package='ros_awsiot_agent',
        executable='mqtt2ros',
        name='mqtt2ros',
        output='screen',
        parameters=[
            {'topic_to': '~/output'},
            {'topic_from': EnvironmentVariable('AWSIOT_MQTT_TOPIC')},
            {'topic_type': 'std_msgs/String'},
            {'retry_wait': 10},
            {'cert': PathJoinSubstitution(
                [EnvironmentVariable('AWSIOT_CERT_DIR'), 'certificate.pem.crt'])},
            {'key': PathJoinSubstitution(
                [EnvironmentVariable('AWSIOT_CERT_DIR'), 'private.pem.key'])},
            {'root_ca': os.path.expanduser(
                '~/.aws/cert_root/AmazonRootCA1.pem')},
            {'endpoint': EnvironmentVariable('AWSIOT_ENDPOINT')},
            {'client_id': [EnvironmentVariable(
                'AWSIOT_THING_NAME'), TextSubstitution(text='-mqtt2ros')]},
            {'signing_region': 'ap-northeast-1'},
            {'use_websocket': False}
        ]
    )

    talker_node = Node(
        package='ros_awsiot_agent',
        executable='talker',
        name='talker',
        output='screen'
    )

    listener_node = Node(
        package='ros_awsiot_agent',
        executable='listener',
        name='listener',
        output='screen'
    )

    return LaunchDescription([
        ros2mqtt_node,
        mqtt2ros_node,
        talker_node,
        listener_node
    ])
