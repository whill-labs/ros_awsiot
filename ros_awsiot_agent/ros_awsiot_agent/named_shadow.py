#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 WHILL Inc.
# SPDX-License-Identifier: MIT

import logging
import time
from os.path import expanduser
from typing import Any, Dict
from uuid import uuid4

from awsiotclient import mqtt, named_shadow
import awscrt.exceptions
import rclpy
from rclpy.node import Node
from ros_awsiot_agent import set_module_logger
from ros_awsiot_agent.mqtt_logging import setup_aws_iot_logging
from rosbridge_library.internal.message_conversion import (
    extract_values,
    populate_instance,
)
from ros2topic.api import get_msg_class


set_module_logger(modname="awsiotclient", level=logging.WARN)
setup_aws_iot_logging()


class ShadowParams:
    def __init__(
        self,
        thing_name: str = "",
        name: str = "",
        enable_downstream: bool = False,
        enable_upstream: bool = True,
        publish_full_doc: bool = False,
        use_desired_as_downstream: bool = True,
        retry_wait: int = 10,
        retry: int = 100,
    ) -> None:
        self.thing_name = thing_name
        self.name = name
        self.enable_downstream = enable_downstream
        self.enable_upstream = enable_upstream
        self.publish_full_doc = publish_full_doc
        self.use_desired_as_downstream = use_desired_as_downstream
        self.retry_wait = retry_wait
        self.retry = retry


class Ros2Shadow(Node):
    def __init__(
        self, conn_params: mqtt.ConnectionParams, shadow_params: ShadowParams
    ) -> None:
        super().__init__('ros2shadow')

        self.declare_parameter('input_topic', '~/input')
        self.declare_parameter('output_topic', '~/output')
        self.declare_parameter('output_topic_type', 'std_msgs/String')

        upstream_topic = self.get_parameter('input_topic').value
        downstream_topic = self.get_parameter('output_topic').value
        downstream_topic_type = self.get_parameter('output_topic_type').value

        upstream_topic_type = get_msg_class(
            self, upstream_topic, blocking=True)
        self.get_logger().info(f"upstream_topic_type: {upstream_topic_type}")

        upstream_topic_class = None
        if shadow_params.enable_upstream:
            upstream_topic_class = self.import_message_type(
                upstream_topic_type)
            self.get_logger().debug(
                f"ROS topic {upstream_topic} ({upstream_topic_type}) detected."
            )

        downstream_topic_class = None
        if shadow_params.enable_downstream:
            downstream_topic_class = self.import_message_type(
                downstream_topic_type)
            self.get_logger().debug(
                f"ROS topic {downstream_topic}({downstream_topic_type}) detected.")

        self.mqtt_connection = mqtt.init(conn_params)
        connected = False
        attempts = 0
        while not connected and attempts < shadow_params.retry:
            try:
                connect_future = self.mqtt_connection.connect()
                connect_future.result()
                self.get_logger().info("Connected to AWS IoT!")
                connected = True
            except awscrt.exceptions.AwsCrtError as e:
                attempts += 1
                if attempts < shadow_params.retry:
                    self.get_logger().warn(
                        f"AWS IoT connection attempt {attempts}/{shadow_params.retry} failed: "
                        f"{e}, retrying in {shadow_params.retry_wait} seconds...")
                    time.sleep(shadow_params.retry_wait)
                else:
                    self.get_logger().warn(
                        f"AWS IoT connection attempt {attempts}/{shadow_params.retry} failed: {e}")
            except Exception as e:
                attempts += 1
                if attempts < shadow_params.retry:
                    self.get_logger().error(
                        f"Unexpected connection error {attempts}/{shadow_params.retry}: "
                        f"{e}, retrying in {shadow_params.retry_wait} seconds...")
                    time.sleep(shadow_params.retry_wait)
                else:
                    self.get_logger().error(
                        f"Unexpected connection error {attempts}/{shadow_params.retry}: {e}")

        if not connected:
            self.get_logger().error(
                f"Failed to connect to AWS IoT after {shadow_params.retry} attempts")
            raise RuntimeError("AWS IoT connection failed")

        # Initialize Publisher
        if downstream_topic_class:
            self.pub = self.create_publisher(
                downstream_topic_class,
                downstream_topic,
                10
            )
            self.downstream_topic_class = downstream_topic_class
            delta_func = self.accept_delta
        else:
            delta_func = self.deny_delta

        self.shadow_cli = named_shadow.client(
            self.mqtt_connection,
            thing_name=shadow_params.thing_name,
            shadow_name=shadow_params.name,
            publish_full_doc=shadow_params.publish_full_doc,
        )

        if shadow_params.use_desired_as_downstream:
            self.get_logger().debug("use desired")
            self.shadow_cli.desired_func = delta_func
        else:
            self.get_logger().debug("use delta")
            self.shadow_cli.delta_func = delta_func

        # Initialize Subscriber
        if upstream_topic_class:
            self.sub = self.create_subscription(
                upstream_topic_class,
                upstream_topic,
                self.callback,
                10
            )

    # Helper method to determine topic type
    def determine_topic_type(self, topic_name):
        try:
            # Get topic information using the node's get_topic_names_and_types
            # method
            topic_names_and_types = self.get_topic_names_and_types()

            # Find an exact match for the topic name
            for name, types in topic_names_and_types:
                if name == topic_name and types:
                    # Return the first type (usually there's only one)
                    self.get_logger().info(
                        f"Detected type for topic {topic_name}: {types[0]}")
                    return types[0]

            self.get_logger().warn(
                f"Could not detect type for topic {topic_name}")
        except Exception as e:
            self.get_logger().error(
                f"Error occurred while detecting topic type: {e}")
            return None

    # Helper method to import message type
    def import_message_type(self, type_info):
        """
        Process message type and return appropriate class

        Args:
            type_info: String type name ('std_msgs/String' etc.) or already imported message class

        Returns:
            Message class, or None if failed
        """
        try:
            # If already a class object, return as is
            if isinstance(type_info, type):
                self.get_logger().debug(
                    f"Message class directly passed: {type_info.__module__}.{type_info.__name__}")
                return type_info

            # If string format, process traditionally
            if isinstance(type_info, str):
                # Handle 'package_name/MessageType' formats
                parts = type_info.split('/')

                # If there are 2 parts, it's in the 'package_name/MessageType'
                if len(parts) == 2:
                    package_name = parts[0]
                    msg_type = parts[1]
                    module_name = f"{package_name}.msg"
                else:
                    self.get_logger().error(
                        f"Invalid message type format: {type_info}")
                    return None

                # Import the module and get the class
                self.get_logger().debug(
                    f"Import attempt: module={module_name}, class={msg_type}")
                module = __import__(module_name, fromlist=[msg_type])
                message_class = getattr(module, msg_type)

                self.get_logger().debug(
                    f"Successfully imported message class {type_info}")
                return message_class

            # If neither class nor string, error
            self.get_logger().error(
                f"Invalid message type format: {type(type_info)} {type_info}")
            return None

        except (ImportError, AttributeError) as e:
            self.get_logger().error(
                f"Error occurred while importing message type {type_info}: {e}")
            # More detailed debug information
            import traceback
            self.get_logger().debug(traceback.format_exc())
            return None

    def accept_delta(
        self, thing_name: str, shadow_name: str, value: Dict[str, Any]
    ) -> None:
        self.get_logger().debug(
            f"cb invoked. thing_name: {thing_name}, shadow_name: {shadow_name}"
        )
        self.get_logger().debug(
            f"value: {value}"
        )
        try:
            downstream_inst = self.downstream_topic_class()
            msg = populate_instance(value, downstream_inst)
        except Exception as e:
            self.get_logger().error(
                f"Failed to create downstream message from shadow value: {e}")
            return
        try:
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish downstream message: {e}")

    def deny_delta(
        self, thing_name: str, shadow_name: str, value: Dict[str, Any]
    ) -> None:
        raise (
            named_shadow.ExceptionAwsIotNamedShadowInvalidDelta(
                "this shadow does not accept any delta"
            )
        )

    def callback(self, msg) -> None:
        try:
            msg_dict = extract_values(msg)
        except Exception as e:
            self.get_logger().error(
                f"Failed to extract values from ROS message: {e}")
            return
        try:
            self.shadow_cli.change_reported_value(msg_dict)
        except awscrt.exceptions.AwsCrtError as e:
            self.get_logger().error(f"AWS IoT shadow update failed: {e}")
        except Exception as e:
            self.get_logger().error(
                f"Unexpected error updating AWS IoT shadow: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)

    node = rclpy.create_node('ros2shadow_params')

    node.declare_parameter('thing_name', '')
    node.declare_parameter('shadow_name', '')
    node.declare_parameter('publish_full_doc', False)
    node.declare_parameter('use_desired_as_downstream', True)
    node.declare_parameter('enable_downstream', False)
    node.declare_parameter('enable_upstream', True)
    node.declare_parameter('retry_wait', 10)
    node.declare_parameter('retry_attempts', 100)
    node.declare_parameter('cert', '~/.aws/cert/certificate.pem.crt')
    node.declare_parameter('key', '~/.aws/cert/private.pem.key')
    node.declare_parameter('root_ca', '~/.aws/cert/AmazonRootCA1.pem')
    node.declare_parameter('endpoint', '')
    node.declare_parameter('client_id', '')

    shadow_params = ShadowParams()
    shadow_params.thing_name = node.get_parameter('thing_name').value
    shadow_params.name = node.get_parameter('shadow_name').value
    shadow_params.publish_full_doc = node.get_parameter(
        'publish_full_doc').value
    shadow_params.use_desired_as_downstream = node.get_parameter(
        'use_desired_as_downstream').value
    shadow_params.enable_downstream = node.get_parameter(
        'enable_downstream').value
    shadow_params.enable_upstream = node.get_parameter('enable_upstream').value
    shadow_params.retry_wait = node.get_parameter('retry_wait').value
    shadow_params.retry = node.get_parameter('retry_attempts').value

    conn_params = mqtt.ConnectionParams()
    conn_params.cert = expanduser(node.get_parameter('cert').value)
    conn_params.key = expanduser(node.get_parameter('key').value)
    conn_params.root_ca = expanduser(node.get_parameter('root_ca').value)
    conn_params.endpoint = node.get_parameter('endpoint').value

    client_id_param = node.get_parameter('client_id').value
    conn_params.client_id = client_id_param if client_id_param else shadow_params.thing_name + \
        "-" + str(uuid4())

    node.destroy_node()

    ros2shadow = Ros2Shadow(conn_params, shadow_params)
    rclpy.spin(ros2shadow)

    ros2shadow.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
