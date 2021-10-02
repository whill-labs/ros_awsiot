#!/usr/bin/python3

import rospy
from rosbridge_library.internal.ros_loader import get_message_class
from rosbridge_library.internal.message_conversion import extract_values, populate_instance
from awsiotclient import *
from os.path import expanduser
from uuid import uuid4

class JobParams():
    def __init__(self, thing_name: str = None) -> None:
        self.thing_name = thing_name

class Jobs2Ros():
    def __init__(self, topic_to: str, topic_type: str, conn_params: mqtt.ConnectionParams, job_params: JobParams) -> None:
        topic_to = rospy.remap_name('~output')
        topic_class = get_message_class(topic_type)
        self.inst = topic_class()
        self.mqtt_connection = mqtt.init(conn_params)
        connect_future = self.mqtt_connection.connect()
        connect_future.result()
        rospy.loginfo('Connected!')

        self.jobs_cli = jobs.client(self.mqtt_connection, thing_name=job_params.thing_name, job_func=self.job_func)
        self.pub = rospy.Publisher(topic_to, topic_class, queue_size=10)
    
    def job_func(self, id, document: dict):
        try:
            if(document.get('data') is None):
                raise(jobs.ExceptionAwsIotJobsUserDefinedFailure('job_id {} is not for me'.format(id)))
            msg = populate_instance(document, self.inst)
            num_sub = self.pub.get_num_connections()
            if(num_sub < 1):
                raise(jobs.ExceptionAwsIotJobsUserDefinedFailure('received job_id is {} but there is only {} subscribers'.format(id, num_sub)))
            self.pub.publish(msg)
        except jobs.ExceptionAwsIotJobsUserDefinedFailure as e:
            rospy.logwarn('user-defined failure in job_func: %s', e)
            raise(e)
        except Exception as e:
            rospy.logerr('something happened in job_func: %s', e)
            raise(e)

    def callback(self, msg):
        msg_dict = extract_values(msg)
        self.shadow_cli.change_reported_value(msg_dict)

def main():
    rospy.init_node("job2ros", anonymous=True)

    topic_type = rospy.get_param("~topic_type", default="std_msgs/String")

    job_params = JobParams()
    job_params.thing_name = rospy.get_param('~thing_name')

    conn_params = mqtt.ConnectionParams()

    conn_params.cert = expanduser(rospy.get_param('~cert', default='~/.aws/cert/certificate.pem.crt'))
    conn_params.key = expanduser(rospy.get_param('~key', default='~/.aws/cert/private.pem.key'))
    conn_params.root_ca = expanduser(rospy.get_param('~root_ca', default='~/.aws/cert/AmazonRootCA1.pem'))

    conn_params.endpoint = rospy.get_param('~endpoint')
    
    conn_params.client_id = rospy.get_param('~client_id', default=job_params.thing_name + "-" + str(uuid4()))
    conn_params.signing_region = rospy.get_param('~signing_region', default='ap-northeast-1')
    conn_params.use_websocket = rospy.get_param('~use_websocket', default=False)


    jobs2ros = Jobs2Ros(topic_type, conn_params, job_params)

    rospy.spin()

if __name__ == "__main__":
    main()