#!/usr/bin/env python3

import rospy


def setup_aws_iot_logging():
    """Setup AWS IoT SDK connection events to output as ROS logging"""
    try:
        from awsiotclient import mqtt as _awsiot_mqtt

        def _ros_on_connection_interrupted(connection, error, **kwargs):  # type: ignore
            rospy.logwarn("AWS IoT connection interrupted: %s", error)

        def _ros_on_connection_resumed(connection, return_code, session_present, **kwargs):  # type: ignore
            rospy.logwarn(
                "AWS IoT connection resumed: return_code=%s session_present=%s",
                return_code,
                session_present,
            )

        _awsiot_mqtt.on_connection_interrupted = _ros_on_connection_interrupted  # type: ignore
        _awsiot_mqtt.on_connection_resumed = _ros_on_connection_resumed  # type: ignore
    except Exception:
        # Ignore failures here as they are not critical
        pass
