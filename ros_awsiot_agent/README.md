# ros_awsiot_agent

This package provides bridge functionalities between ROS and AWS IoT.

## Requirements

- python3.6+
- ROS Noetic
- [awsiotclient 0.1.1+](https://pypi.org/project/awsiotclient/) (Note: awsiotclient cannot be installed via `rosdep install`. Install the package via pip.)

## Usage

### Named Shadow

named_shadow.py provides two-way communication from/to ROS to/from AWS IoT thing shadow.

#### Sample launch file

```xml
<launch>
  <node pkg="ros_awsiot_agent" type="named_shadow.py" name="named_shadow" output="screen" >
    <param name="~endpoint" value="your_end_point_url"/>
    <param name="~thing_name" value="your_thing_name"/>
    <param name="~root_ca" value="path_to_root_ca_certificate"/>
    <param name="~cert" value="path_to_your_certificate"/>
    <param name="~key" value="path_to_your_private_key"/>
    <param name="~shadow_name" value="your_shadow_name"/>
    <param name="~publish_full_doc" value="True"/>
    <param name="~enable_upstream" value="True"/>
    <param name="~enable_downstream" value="True"/>
  </node>
</launch>
```

#### Subscribed Topics

- `~input` (any message type)
: Upstream (ROS -> AWS IoT) message. The node translates the contents of this message to JSON and sends it to shadow as "reported" document. Note that its message type is determined by its **publisher**.

#### Published Topics

- `~output` (any message type): Downstream (AWS IoT -> ROS) message. When the node receives **delta** document from shadow, the node translate the JSON document to equivalent ROS message and publishes it. Note that its message type is determined by its **subscriber**. If the message structure is not compatible with the received JSON, the node outputs error.

#### Parameters

- `~endpoint` (string): Endpoint URL for your AWS IoT Core. (required)
- `~thing_name` (string): Name of the AWS IoT Thing. (required)
- `~shadow_name` (string): Name of your thing shadow. (required)
- `~client_id` (string): ID of MQTT client. (optional. default=`<thing_name>-str(uuid4())`)
- `~root_ca` (string): Path to RootCA certificate. (optional. default=`~/.aws/cert/AmazonRootCA1.pem`)
- `~cert` (string): Path to your thing's certificate. (optional. default=`~/.aws/cert/certificate.pem.crt`)
- `~key` (string): Path to your thing's private key. (optional. default=`~/.aws/cert/private.pem.key`)
- `~publish_full_doc` (bool): If false, only updated snippet of shadow document is published. (optional. default=`False`)
- `~enable_upstream` (bool): If true, `~input` topic is enabled. (optional. default=`True`)
- `~enable_downstream` (bool): If true, `~output` topic is enabled. (optional. default=`False`)

## License

This library is licensed under the MIT license.

## Acknowledgements

- [groove-x/mqtt_bridge](https://github.com/groove-x/mqtt_bridge)
