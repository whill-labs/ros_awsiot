#!/usr/bin/python3

import logging
from os.path import expanduser
from uuid import uuid4

import rospy
from awsiotclient import mqtt, pubsub
from ros_awsiot_agent import set_module_logger
from rosbridge_library.internal.message_conversion import extract_values
from rostopic import ROSTopicIOException, get_topic_class, get_topic_type

set_module_logger(modname="awsiotclient", level=logging.WARN)


class Ros2Mqtt:
    def __init__(
        self, topic_from: str, topic_to: str, conn_params: mqtt.ConnectionParams
    ) -> None:
        topic_class = None
        while topic_class is None:
            try:
                topic_class, _, _ = get_topic_class(topic_from)
                topic_type = get_topic_type(topic_from)
                rospy.loginfo("ROS topic %s (%s) detected.", topic_from, topic_type)
            except ROSTopicIOException as e:
                rospy.loginfo(
                    "ROS topic %s is not ready yet. %s raised.", topic_from, e
                )
            rospy.sleep(1.0)
        self.inst = topic_class()
        self.mqtt_connection = mqtt.init(conn_params)
        connect_future = self.mqtt_connection.connect()
        connect_future.result()
        rospy.loginfo("Connected!")

        self.mqtt_pub = pubsub.Publisher(self.mqtt_connection, topic_to)
        self.sub = rospy.Subscriber(topic_from, topic_class, callback=self.callback)

    def callback(self, msg):
        msg_dict = extract_values(msg)
        self.mqtt_pub.publish(msg_dict)


def main():
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
    thing_name = rospy.get_param("~thing_name", default="NOT-A-THING")

    conn_params.client_id = rospy.get_param(
        "~client_id", default="ros2mqtt-" + str(uuid4())
    )
    conn_params.signing_region = rospy.get_param(
        "~signing_region", default="ap-northeast-1"
    )
    conn_params.use_websocket = rospy.get_param("~use_websocket", default=False)

    ros2mqtt = Ros2Mqtt(topic_from, topic_to, conn_params)
    rospy.spin()


if __name__ == "__main__":
    main()
