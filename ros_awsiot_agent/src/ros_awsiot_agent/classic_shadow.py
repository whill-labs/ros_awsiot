#!/usr/bin/env python3

import logging
from os.path import expanduser
from uuid import uuid4

import rospy
from awsiotclient import classic_shadow, mqtt
from ros_awsiot_agent import set_module_logger
from rosbridge_library.internal.message_conversion import (
    extract_values,
    populate_instance,
)
from rostopic import ROSTopicIOException, get_topic_class, get_topic_type

set_module_logger(modname="awsiotclient", level=logging.WARN)


class ShadowParams:
    def __init__(
        self,
        thing_name: str = None,
        name: str = None,
        accept_delta: bool = False,
        publish_full_doc: bool = False,
    ) -> None:
        self.thing_name = thing_name
        self.name = name
        self.accept_delta = accept_delta
        self.publish_full_doc = publish_full_doc


class Ros2Shadow:
    def __init__(
        self, conn_params: mqtt.ConnectionParams, shadow_params: ShadowParams
    ) -> None:
        topic_from = rospy.remap_name("~input")
        topic_to = rospy.remap_name("~output")

        topic_class = None
        while topic_class is None:
            try:
                topic_class, _, _ = get_topic_class(topic_from)
                topic_type = get_topic_type(topic_from)
                rospy.logdebug("ROS topic %s (%s) detected.", topic_from, topic_type)
            except ROSTopicIOException as e:
                rospy.logdebug(
                    "ROS topic %s is not ready yet. %s raised.", topic_from, e
                )
            rospy.sleep(1.0)
        self.inst = topic_class()
        self.mqtt_connection = mqtt.init(conn_params)
        connect_future = self.mqtt_connection.connect()
        connect_future.result()
        rospy.logdebug("Connected!")

        if shadow_params.accept_delta:
            delta_func = self.accept_delta
        else:
            delta_func = self.deny_delta

        self.shadow_cli = classic_shadow.client(
            self.mqtt_connection,
            thing_name=conn_params.thing_name,
            shadow_property=shadow_params.name,
            delta_func=delta_func,
        )
        self.sub = rospy.Subscriber(topic_from, topic_class, callback=self.callback)
        self.pub = rospy.Publisher(topic_to, topic_class, queue_size=10)

    def accept_delta(self, thing_name: str, shadow_name: str, value: dict):
        msg = populate_instance(value, self.inst)
        self.pub.publish(msg)

    def deny_delta(self, thing_name: str, shadow_name: str, value: dict):
        raise (
            classic_shadow.ExceptionAwsIotClassicShadowInvalidDelta(
                "this shadow does not accept any delta"
            )
        )

    def callback(self, msg):
        msg_dict = extract_values(msg)
        self.shadow_cli.change_reported_value(msg_dict)


def main():
    rospy.init_node("ros2shadow", anonymous=True)

    shadow_params = ShadowParams()
    shadow_params.thing_name = rospy.get_param("~thing_name")
    shadow_params.name = rospy.get_param("~shadow_name")
    shadow_params.publish_full_doc = rospy.get_param("~publish_full_doc", default=False)
    shadow_params.accept_delta = rospy.get_param("~accept_delta", default=False)

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
        "~client_id", default=shadow_params.thing_name + "-" + str(uuid4())
    )
    conn_params.signing_region = rospy.get_param(
        "~signing_region", default="ap-northeast-1"
    )
    conn_params.use_websocket = rospy.get_param("~use_websocket", default=False)

    ros2shadow = Ros2Shadow(conn_params, shadow_params)
    rospy.spin()


if __name__ == "__main__":
    main()
