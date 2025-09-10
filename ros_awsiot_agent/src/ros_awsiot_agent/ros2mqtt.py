#!/usr/bin/env python3

import logging
from os.path import expanduser
from uuid import uuid4

import rospy
from awsiotclient import mqtt, pubsub
import awscrt.exceptions
from ros_awsiot_agent import set_module_logger
from rosbridge_library.internal.message_conversion import extract_values
from rostopic import ROSTopicIOException, get_topic_class, get_topic_type

from .mqtt_logging import setup_aws_iot_logging

set_module_logger(modname="awsiotclient", level=logging.WARN)
setup_aws_iot_logging()


class Ros2Mqtt:
    def __init__(
        self,
        topic_from: str,
        topic_to: str,
        conn_params: mqtt.ConnectionParams,
        retry_wait: float = 10.0,
        retry: int = 100,
    ) -> None:
        topic_class = None
        now = rospy.Time.now()
        timeout = rospy.Duration.from_sec(60.0)
        timediff = rospy.Duration.from_sec(0.0)
        while topic_class is None and timediff < timeout:  # type: ignore
            try:
                topic_class, _, _ = get_topic_class(topic_from)
                topic_type = get_topic_type(topic_from)
                rospy.loginfo("ROS topic %s (%s) detected.", topic_from, topic_type)
            except ROSTopicIOException as e:
                rospy.loginfo(
                    "ROS topic %s is not ready yet. %s raised.", topic_from, e
                )
            rospy.sleep(1.0)
            timediff = rospy.Time.now() - now

        self.mqtt_connection = mqtt.init(conn_params)
        connected = False
        attempts = 0
        while not connected and attempts < retry:
            try:
                connect_future = self.mqtt_connection.connect()
                connect_future.result()
                rospy.loginfo("Connected to AWS IoT!")
                connected = True
            except awscrt.exceptions.AwsCrtError as e:
                attempts += 1
                rospy.logwarn("AWS IoT connection attempt %d/%d failed: %s, retrying in %s seconds...", attempts, retry, e, retry_wait)
                if attempts < retry:
                    rospy.sleep(retry_wait)
            except Exception as e:
                attempts += 1
                rospy.logerr("Unexpected connection error %d/%d: %s, retrying in %s seconds...", attempts, retry, e, retry_wait)
                if attempts < retry:
                    rospy.sleep(retry_wait)

        if not connected:
            rospy.logerr("Failed to connect to AWS IoT after %d attempts", retry)
            raise RuntimeError("AWS IoT connection failed")

        self.mqtt_pub = pubsub.Publisher(self.mqtt_connection, topic_to)
        self.sub = rospy.Subscriber(topic_from, topic_class, callback=self.callback)

    def callback(self, msg: rospy.AnyMsg) -> None:
        try:
            msg_dict = extract_values(msg)
        except Exception as e:
            rospy.logerr("Failed to extract values from ROS message: %s", e)
            return

        try:
            self.mqtt_pub.publish(msg_dict)
        except awscrt.exceptions.AwsCrtError as e:
            rospy.logerr("AWS IoT publish failed: %s", e)
        except (TypeError, ValueError) as e:
            rospy.logerr("Invalid message format for MQTT publish: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error publishing to AWS IoT: %s", e)


def main() -> None:
    rospy.init_node("ros2mqtt", anonymous=True)

    topic_from = rospy.get_param("~topic_from", default="~input")
    topic_to = rospy.get_param("~topic_to", default="/ros2mqtt")

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
        "~client_id", default="ros2mqtt-" + str(uuid4())
    )
    conn_params.signing_region = rospy.get_param(
        "~signing_region", default="ap-northeast-1"
    )
    conn_params.use_websocket = rospy.get_param("~use_websocket", default=False)

    retry_wait = rospy.get_param("~retry_wait", default=10.0)
    retry = rospy.get_param("~retry_attempts", default=100)

    Ros2Mqtt(topic_from, topic_to, conn_params, retry_wait, retry)
    rospy.spin()


if __name__ == "__main__":
    main()
