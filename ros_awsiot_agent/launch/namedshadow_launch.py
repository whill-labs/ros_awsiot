#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    thing_name = EnvironmentVariable('AWSIOT_THING_NAME')
    endpoint = EnvironmentVariable('AWSIOT_ENDPOINT')
    cert = PathJoinSubstitution(
        [EnvironmentVariable('AWSIOT_CERT_DIR'), 'certificate.pem.crt'])
    key = PathJoinSubstitution(
        [EnvironmentVariable('AWSIOT_CERT_DIR'), 'private.pem.key'])

    if thing_name is None:
        raise ValueError('AWSIOT_THING_NAME is not set')
    if endpoint is None:
        raise ValueError('AWSIOT_ENDPOINT is not set')
    if cert is None:
        raise ValueError('AWSIOT_CERT_DIR is not set')
    if key is None:
        raise ValueError('AWSIOT_CERT_DIR is not set')

    return LaunchDescription([
        # Named Shadow Node
        Node(
            package='ros_awsiot_agent',
            executable='named_shadow',
            name='named_shadow',
            output='screen',
            parameters=[{
                'thing_name': EnvironmentVariable('AWSIOT_THING_NAME'),
                'endpoint': EnvironmentVariable('AWSIOT_ENDPOINT'),
                'root_ca': '~/.aws/cert_root/AmazonRootCA1.pem',
                'cert': PathJoinSubstitution([EnvironmentVariable('AWSIOT_CERT_DIR'), 'certificate.pem.crt']),
                'key': PathJoinSubstitution([EnvironmentVariable('AWSIOT_CERT_DIR'), 'private.pem.key']),
                'shadow_name': 'test',
                'publish_full_doc': True,
                'use_desired_as_downstream': True,
                'enable_upstream': True,
                'enable_downstream': True,
                'input_topic': 'upstream',
                'output_topic': 'downstream',
                'output_topic_type': 'std_msgs/String',
            }],
        ),

        # Talker Node
        Node(
            package='ros_awsiot_agent',
            executable='talker',
            name='talker',
            remappings=[
                ('chatter', 'upstream'),
            ],
        ),

        # Listener Node
        Node(
            package='ros_awsiot_agent',
            executable='listener',
            name='listener',
            output='screen',
            remappings=[
                ('chatter', 'downstream'),
            ],
        ),
    ])
