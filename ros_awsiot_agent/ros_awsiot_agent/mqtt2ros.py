#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 WHILL Inc.
# SPDX-License-Identifier: MIT

import logging
import gzip
import json
from os.path import expanduser
from typing import Any, Dict
from uuid import uuid4
import time

import rclpy
from rclpy.node import Node
from awsiotclient import mqtt, pubsub
from ros_awsiot_agent import set_module_logger
from ros_awsiot_agent.mqtt_logging import setup_aws_iot_logging
from rosidl_runtime_py.utilities import get_message
import awscrt.exceptions
from awscrt.mqtt import QoS
from ros_awsiot_agent.message_conversion import populate_instance

set_module_logger(modname="awsiotclient", level=logging.WARN)
setup_aws_iot_logging()


class Mqtt2Ros(Node):
    def __init__(
        self,
        topic_from: str,
        topic_to: str,
        topic_type: str,
        conn_params: mqtt.ConnectionParams,
        retry_wait: int,
        use_gzip_compression: bool,
        max_attempts: int = 100
    ) -> None:
        super().__init__('mqtt2ros')

        topic_class = get_message(topic_type)
        self.inst = topic_class()
        self.mqtt_connection = mqtt.init(conn_params)

        connected = False
        attempts = 0
        while not connected and attempts < max_attempts:
            try:
                connect_future = self.mqtt_connection.connect()
                connect_future.result()
                self.get_logger().info("Connected to AWS IoT!")
                connected = True
            except awscrt.exceptions.AwsCrtError as e:
                attempts += 1
                self.get_logger().warn(
                    f"AWS IoT connection attempt {attempts}/{max_attempts} failed: "
                    f"{e}, retrying in {retry_wait} seconds...")
                if attempts < max_attempts:
                    time.sleep(retry_wait)

        if not connected:
            self.get_logger().error(
                f"Failed to connect to AWS IoT after {max_attempts} attempts")
            raise RuntimeError("AWS IoT connection failed")

        # create ROS2 publisher
        self.pub = self.create_publisher(topic_class, topic_to, 10)
        try:
            if use_gzip_compression:
                self.mqtt_sub = self.mqtt_connection.subscribe(
                    topic_from, callback=self.callback_gzip_compression, qos=QoS.AT_LEAST_ONCE
                )
            else:
                self.mqtt_sub = pubsub.Subscriber(
                    self.mqtt_connection, topic_from, callback=self.callback
                )
        except awscrt.exceptions.AwsCrtError as e:
            self.get_logger().error(
                f"AWS IoT subscription failed for topic '{topic_from}': {e}")
            raise
        except Exception as e:
            self.get_logger().error(
                f"Unexpected error subscribing to topic '{topic_from}': {e}")
            raise

    def callback(self, topic: str, msg_dict: Dict[str, Any]) -> None:
        self.get_logger().info(f"Received message: {msg_dict}")
        try:
            msg = populate_instance(msg_dict, self.inst)
        except Exception as e:
            self.get_logger().error(
                f"Failed to convert MQTT message to ROS message: {e}")
            return
        try:
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish ROS message: {e}")

    def callback_gzip_compression(self, topic: str, payload: bytes) -> None:
        self.get_logger().info(f"Received gzip compressed message: {payload}")
        try:
            msg_dict = json.loads(gzip.decompress(payload).decode('utf-8'))
            msg = populate_instance(msg_dict, self.inst)
        except Exception as e:
            self.get_logger().error(
                f"Failed to decompress/convert MQTT message to ROS message: {e}")
            return
        try:
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish ROS message: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)

    node = rclpy.create_node('mqtt2ros_param_node')

    node.declare_parameter('topic_to', 'output')
    node.declare_parameter('topic_from', '/mqtt2ros')
    node.declare_parameter('topic_type', 'std_msgs/String')
    node.declare_parameter('retry_wait', 10)
    node.declare_parameter('max_attempts', 100)
    node.declare_parameter('use_gzip_compression', False)

    topic_to = node.get_parameter('topic_to').value
    topic_from = node.get_parameter('topic_from').value
    topic_type = node.get_parameter('topic_type').value
    retry_wait = node.get_parameter('retry_wait').value
    max_attempts = node.get_parameter('max_attempts').value
    use_gzip_compression = node.get_parameter('use_gzip_compression').value
    if topic_type is None:
        node.get_logger().error("topic_type is not specified")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.declare_parameter('cert', '~/.aws/cert/certificate.pem.crt')
    node.declare_parameter('key', '~/.aws/cert/private.pem.key')
    node.declare_parameter('root_ca', '~/.aws/cert/AmazonRootCA1.pem')
    node.declare_parameter('endpoint', '')
    node.declare_parameter('client_id', 'mqtt-' + str(uuid4()))
    node.declare_parameter('signing_region', 'ap-northeast-1')
    node.declare_parameter('use_websocket', False)

    conn_params = mqtt.ConnectionParams()
    conn_params.cert = expanduser(node.get_parameter('cert').value)
    conn_params.key = expanduser(node.get_parameter('key').value)
    conn_params.root_ca = expanduser(node.get_parameter('root_ca').value)
    conn_params.endpoint = node.get_parameter('endpoint').value
    conn_params.client_id = node.get_parameter('client_id').value
    conn_params.signing_region = node.get_parameter('signing_region').value
    conn_params.use_websocket = node.get_parameter('use_websocket').value

    node.destroy_node()

    mqtt2ros_node = Mqtt2Ros(
        topic_from,
        topic_to,
        topic_type,
        conn_params,
        retry_wait,
        use_gzip_compression,
        max_attempts)

    try:
        rclpy.spin(mqtt2ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        mqtt2ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
