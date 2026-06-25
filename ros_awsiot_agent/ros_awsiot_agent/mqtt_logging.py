#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 WHILL Inc.
# SPDX-License-Identifier: MIT

import rclpy.logging


def setup_aws_iot_logging():
    """Emit AWS IoT SDK connection interrupted/resumed events as ROS WARN logs.

    The default callbacks registered by awsiotclient.mqtt log interrupted/resumed
    events at debug level, so connection drops/recoveries are not recorded at the
    usual log level. This replaces them (monkey-patch) with callbacks that log via
    the rclpy logger at WARN. Because awsiotclient.mqtt.init() resolves the
    callbacks by their module-global names at call time, this function must run
    before init() is called (i.e. at each node's import time).

    The resubscribe handling on resume is preserved by delegating to the default
    callback.
    """
    logger = rclpy.logging.get_logger("awsiot_mqtt")

    try:
        from awsiotclient import mqtt as _awsiot_mqtt
    except Exception as e:
        logger.debug(f"Failed to setup AWS IoT logging: {e}")
        return

    # Keep the default callback to preserve its resubscribe handling
    # (resubscribe when the session is not persisted on resume).
    _default_on_connection_resumed = _awsiot_mqtt.on_connection_resumed

    def _ros_on_connection_interrupted(connection, error, **kwargs):
        logger.warn(f"AWS IoT connection interrupted: {error}")

    def _ros_on_connection_resumed(connection, return_code, session_present, **kwargs):
        logger.warn(
            f"AWS IoT connection resumed: return_code={return_code} "
            f"session_present={session_present}"
        )
        # Delegate to the default callback to keep its resubscribe handling.
        _default_on_connection_resumed(connection, return_code, session_present, **kwargs)

    _awsiot_mqtt.on_connection_interrupted = _ros_on_connection_interrupted
    _awsiot_mqtt.on_connection_resumed = _ros_on_connection_resumed
