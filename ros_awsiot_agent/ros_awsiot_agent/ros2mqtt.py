#!/usr/bin/env python3

import logging
import time
from os.path import expanduser
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from awsiotclient import mqtt, pubsub
import awscrt.exceptions
from ros_awsiot_agent import set_module_logger
from ros_awsiot_agent.message_conversion import extract_values
from rclpy.callback_groups import ReentrantCallbackGroup
from ros2topic.api import get_msg_class

set_module_logger(modname="awsiotclient", level=logging.DEBUG)


class Ros2Mqtt:
    def __init__(
        self, node: Node, topic_from: str, topic_to: str, msg_type, conn_params: mqtt.ConnectionParams
    ) -> None:
        self.node = node

        # MQTT接続
        self.mqtt_connection = mqtt.init(conn_params)
        connected = False
        while not connected:
            try:
                connect_future = self.mqtt_connection.connect()
                connect_future.result()
                self.node.get_logger().info("Connected to AWS IoT!")
                connected = True
            except awscrt.exceptions.AwsCrtError as e:
                self.node.get_logger().warn(
                    f"Connection attempt failed: {e}, retrying in 10 seconds...")
                time.sleep(10)

        self.mqtt_pub = pubsub.Publisher(self.mqtt_connection, topic_to)

        qos = QoSProfile(depth=10)
        self.sub = self.node.create_subscription(
            msg_type,
            topic_from,
            self.callback,
            qos,
            callback_group=ReentrantCallbackGroup()
        )

    def callback(self, msg) -> None:
        msg_dict = extract_values(msg)
        self.mqtt_pub.publish(msg_dict)


def main(args=None) -> None:
    rclpy.init(args=args)

    # 一つのノードだけを作成
    node = rclpy.create_node(
        'ros2mqtt',
        allow_undeclared_parameters=True)

    node.declare_parameter('topic_from', 'input')
    node.declare_parameter('topic_to', '/ros2mqtt')
    node.declare_parameter('cert', '~/.aws/cert/certificate.pem.crt')
    node.declare_parameter('key', '~/.aws/cert/private.pem.key')
    node.declare_parameter('root_ca', '~/.aws/cert/AmazonRootCA1.pem')
    node.declare_parameter('endpoint', '')
    node.declare_parameter('client_id', f'ros2mqtt-{str(uuid4())}')
    node.declare_parameter('signing_region', 'ap-northeast-1')
    node.declare_parameter('use_websocket', False)

    topic_from = node.get_parameter('topic_from').value
    topic_to = node.get_parameter('topic_to').value

    msg_type = get_msg_class(node, topic_from, blocking=True)
    if msg_type is None:
        node.get_logger().error(
            f"Could not determine message type for {topic_from} after {timeout} seconds")
        node.destroy_node()
        rclpy.shutdown()
        return

    conn_params = mqtt.ConnectionParams()
    conn_params.cert = node.get_parameter('cert').value
    conn_params.key = node.get_parameter('key').value
    conn_params.root_ca = node.get_parameter('root_ca').value
    conn_params.endpoint = node.get_parameter('endpoint').value
    conn_params.client_id = node.get_parameter('client_id').value
    conn_params.signing_region = node.get_parameter('signing_region').value
    conn_params.use_websocket = node.get_parameter('use_websocket').value

    Ros2Mqtt(node, topic_from, topic_to, msg_type, conn_params)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
