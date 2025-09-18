#!/usr/bin/env python3

import logging
from os.path import expanduser
from typing import Any, Dict
from uuid import uuid4

import rospy
from awsiotclient import mqtt, pubsub
from ros_awsiot_agent import set_module_logger
from ros_awsiot_agent.mqtt_logging import setup_aws_iot_logging
from rosbridge_library.internal.ros_loader import get_message_class
import awscrt.exceptions

from message_conversion import populate_instance

set_module_logger(modname="awsiotclient", level=logging.WARN)
setup_aws_iot_logging()


class Mqtt2Ros:
    def __init__(
        self,
        topic_from: str,
        topic_to: str,
        topic_type: str,
        conn_params: mqtt.ConnectionParams,
        retry_wait: float = 10.0,
        max_attempts: int = 100,
    ) -> None:
        topic_class = get_message_class(topic_type)
        self.inst = topic_class()
        self.mqtt_connection = mqtt.init(conn_params)

        connected = False
        attempts = 0
        while not connected and attempts < max_attempts:
            try:
                connect_future = self.mqtt_connection.connect()
                connect_future.result()
                rospy.loginfo("Connected to AWS IoT!")
                connected = True
            except awscrt.exceptions.AwsCrtError as e:
                attempts += 1
                rospy.logwarn("Connection attempt %d/%d failed: %s, retrying in %s seconds...", attempts, max_attempts, e, retry_wait)
                if attempts < max_attempts:
                    rospy.sleep(retry_wait)

        if not connected:
            rospy.logerr("Failed to connect to AWS IoT after %d attempts", max_attempts)
            raise RuntimeError("AWS IoT connection failed")

        self.pub = rospy.Publisher(topic_to, topic_class, queue_size=10)
        try:
            self.mqtt_sub = pubsub.Subscriber(
                self.mqtt_connection, topic_from, callback=self.callback
            )
        except awscrt.exceptions.AwsCrtError as e:
            rospy.logerr("AWS IoT subscription failed for topic '%s': %s", topic_from, e)
            raise
        except ValueError as e:
            rospy.logerr("Invalid topic name '%s': %s", topic_from, e)
            raise
        except Exception as e:
            rospy.logerr("Unexpected error subscribing to topic '%s': %s", topic_from, e)
            raise

    def callback(self, topic: str, msg_dict: Dict[str, Any]) -> None:
        try:
            msg = populate_instance(msg_dict, self.inst)
        except Exception as e:
            rospy.logerr("Failed to convert MQTT message to ROS message: %s", e)
            return

        try:
            self.pub.publish(msg)
        except Exception as e:
            rospy.logerr("Failed to publish ROS message: %s", e)


def main() -> None:
    rospy.init_node("mqtt2ros", anonymous=True)

    topic_to = rospy.get_param("~topic_to", default="~output")
    topic_from = rospy.get_param("~topic_from", default="/mqtt2ros")
    topic_type = rospy.get_param("~topic_type", default="std_msgs/String")
    retry_wait = rospy.get_param("~retry_wait", default=10.0)
    max_attempts = rospy.get_param("~max_attempts", default=100)

    conn_params = mqtt.ConnectionParams()

    conn_params.cert = expanduser(
        rospy.get_param("~cert", default="~/.aws/cert/certificate.pem.crt")
    )
    conn_params.key = expanduser(
        rospy.get_param("~key", default="~/.aws/cert/private.pem.key")
    )
    conn_params.root_ca = expanduser(
        rospy.get_param("~root_ca", default="~/.aws/cert/AmazonRootCA1.pem")
    )

    conn_params.endpoint = rospy.get_param("~endpoint")

    conn_params.client_id = rospy.get_param(
        "~client_id", default="mqtt-" + str(uuid4())
    )
    conn_params.signing_region = rospy.get_param(
        "~signing_region", default="ap-northeast-1"
    )
    conn_params.use_websocket = rospy.get_param("~use_websocket", default=False)

    Mqtt2Ros(topic_from, topic_to, topic_type, conn_params, retry_wait, max_attempts)
    rospy.spin()


if __name__ == "__main__":
    main()
