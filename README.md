# ros_awsiot_agent

This package provides bridge functionalities between ROS2 and AWS IoT.

## Requirements

- Python 3.12++
- ROS2 Jazzy Jalisco
- [awsiotclient 0.2.1](https://pypi.org/project/awsiotclient/) (Note: awsiotclient cannot be installed with `rosdep`. Please install it using pip)

## Usage

### Named Shadow

named_shadow.py provides bidirectional communication from ROS2 to AWS IoT named shadows.

#### Sample Launch File

```python
# launch/named_shadow_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_awsiot_agent',
            executable='named_shadow_node.py',
            name='named_shadow',
            output='screen',
            parameters=[
                {'endpoint': 'your_end_point_url'},
                {'thing_name': 'your_thing_name'},
                {'root_ca': 'path_to_root_ca_certificate'},
                {'cert': 'path_to_your_certificate'},
                {'key': 'path_to_your_private_key'},
                {'shadow_name': 'your_shadow_name'},
                {'publish_full_doc': True},
                {'enable_upstream': True},
                {'enable_downstream': True},
                {'input_topic': '/input'},
                {'output_topic': '/output'}
            ]
        )
    ])
```

#### Subscribed Topics

- Topic specified by `input_topic` parameter (default: `/input`) (any message type)
  : Upstream (ROS2 → AWS IoT) messages. The node converts the contents of this message to JSON and sends it as a "reported" document to the shadow. Note that the message type is determined by the **publisher**.

#### Published Topics

- Topic specified by `output_topic` parameter (default: `/output`) (any message type)
  : Downstream (AWS IoT → ROS2) messages. When the node receives a **delta** document from the shadow, the node converts the JSON document to an equivalent ROS2 message and publishes it. Note that the message type is determined by the **subscriber**. If the message structure is not compatible with the received JSON, the node will output an error.

#### Parameters

- `endpoint` (string): AWS IoT Core endpoint URL. (Required)
- `thing_name` (string): AWS IoT Thing name. (Required)
- `shadow_name` (string): Named shadow name. (Required)
- `client_id` (string): MQTT client ID. (Optional. Default=`<thing_name>-str(uuid4())`)
- `root_ca` (string): Path to the RootCA certificate. (Optional. Default=`~/.aws/cert/AmazonRootCA1.pem`)
- `cert` (string): Path to the Thing certificate. (Optional. Default=`~/.aws/cert/certificate.pem.crt`)
- `key` (string): Path to the Thing private key. (Optional. Default=`~/.aws/cert/private.pem.key`)
- `publish_full_doc` (bool): If false, only the updated part of the shadow document is published. (Optional. Default=`False`)
- `enable_upstream` (bool): If true, the `input_topic` topic is enabled. (Optional. Default=`True`)
- `enable_downstream` (bool): If true, the `output_topic` topic is enabled. (Optional. Default=`False`)
- `input_topic` (string): Topic to receive upstream messages. (Optional. Default=`/input`)
- `output_topic` (string): Topic to publish downstream messages. (Optional. Default=`/output`)
- `retry_wait` (int): Wait time (seconds) between connection attempts. (Optional. Default=`10`)

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-repo/ros_awsiot_agent.git
cd ..
colcon build --packages-select ros_awsiot_agent
source install/setup.bash
```

## License

This library is licensed under the MIT License.

## Acknowledgements

- [groove-x/mqtt_bridge](https://github.com/groove-x/mqtt_bridge)
