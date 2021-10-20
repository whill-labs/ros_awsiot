#!/usr/bin/env python3

import logging
from os.path import expanduser
from typing import Any, Dict
from uuid import uuid4

import rospy
from awsiotclient import mqtt, named_shadow
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
        thing_name: str = "",
        name: str = "",
        enable_downstream: bool = False,
        enable_upstream: bool = True,
        publish_full_doc: bool = False,
        use_desired_as_downstream: bool = False,
    ) -> None:
        self.thing_name = thing_name
        self.name = name
        self.enable_downstream = enable_downstream
        self.enable_upstream = enable_upstream
        self.publish_full_doc = publish_full_doc
        self.use_desired_as_downstream = use_desired_as_downstream


class Ros2Shadow:
    def __init__(
        self, conn_params: mqtt.ConnectionParams, shadow_params: ShadowParams
    ) -> None:
        upstream_topic = rospy.resolve_name(rospy.remap_name("~input"))
        downstream_topic = rospy.resolve_name(rospy.remap_name("~output"))

        upstream_topic_class = None
        if shadow_params.enable_upstream:
            while upstream_topic_class is None:
                try:
                    upstream_topic_class, _, _ = get_topic_class(upstream_topic)
                    upstream_topic_type = get_topic_type(upstream_topic)
                    rospy.logdebug(
                        "ROS topic %s (%s) detected.",
                        upstream_topic,
                        upstream_topic_type,
                    )
                except ROSTopicIOException as e:
                    rospy.logdebug(
                        "ROS topic %s is not ready yet. %s raised.", upstream_topic, e
                    )
                rospy.sleep(1.0)

        downstream_topic_class = None
        if shadow_params.enable_downstream:
            while downstream_topic_class is None:
                try:
                    downstream_topic_class, _, _ = get_topic_class(downstream_topic)
                    downstream_topic_type = get_topic_type(downstream_topic)
                    rospy.logdebug(
                        "ROS topic %s (%s) detected.",
                        downstream_topic,
                        downstream_topic_type,
                    )
                except ROSTopicIOException as e:
                    rospy.logdebug(
                        "ROS topic %s is not ready yet. %s raised.", downstream_topic, e
                    )
                rospy.sleep(1.0)

        self.mqtt_connection = mqtt.init(conn_params)
        connect_future = self.mqtt_connection.connect()
        connect_future.result()
        rospy.logdebug("Connected!")

        # Publisher must be initialized before delta_func is registerd to shadow client
        if downstream_topic_class:
            self.pub = rospy.Publisher(
                downstream_topic, downstream_topic_class, queue_size=10
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
            rospy.logdebug("use desired")
            self.shadow_cli.desired_func = delta_func
        else:
            rospy.logdebug("use delta")
            self.shadow_cli.delta_func = delta_func

        # Subscriber must be initialized after shadow client gets ready
        if upstream_topic_class:
            self.sub = rospy.Subscriber(
                upstream_topic, upstream_topic_class, callback=self.callback
            )

    def accept_delta(
        self, thing_name: str, shadow_name: str, value: Dict[str, Any]
    ) -> None:
        rospy.logdebug(
            f"cb invoked. thing_name: {thing_name}, shadow_name: {shadow_name}"
        )
        rospy.logdebug(
            f"value: {value}"
        )
        downstream_inst = self.downstream_topic_class()
        msg = populate_instance(value, downstream_inst)
        self.pub.publish(msg)

    def deny_delta(
        self, thing_name: str, shadow_name: str, value: Dict[str, Any]
    ) -> None:
        raise (
            named_shadow.ExceptionAwsIotNamedShadowInvalidDelta(
                "this shadow does not accept any delta"
            )
        )

    def callback(self, msg: rospy.AnyMsg) -> None:
        msg_dict = extract_values(msg)
        self.shadow_cli.change_reported_value(msg_dict)


def main() -> None:
    rospy.init_node("ros2shadow", anonymous=True)

    shadow_params = ShadowParams()
    shadow_params.thing_name = rospy.get_param("~thing_name")
    shadow_params.name = rospy.get_param("~shadow_name")
    shadow_params.publish_full_doc = rospy.get_param("~publish_full_doc", default=False)
    shadow_params.use_desired_as_downstream = rospy.get_param("~use_desired_as_downstream", default=False)
    shadow_params.enable_downstream = rospy.get_param(
        "~enable_downstream", default=False
    )
    shadow_params.enable_upstream = rospy.get_param("~enable_upstream", default=True)

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

    # Note: signing_region and use_websocket parameters haven't been tested.

    # conn_params.signing_region = rospy.get_param(
    #     "~signing_region", default="ap-northeast-1"
    # )
    # conn_params.use_websocket = rospy.get_param("~use_websocket", default=False)

    Ros2Shadow(conn_params, shadow_params)
    rospy.spin()


if __name__ == "__main__":
    main()
