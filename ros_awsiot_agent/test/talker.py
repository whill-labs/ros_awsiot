#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Header


def talker():
    pub = rospy.Publisher("chatter", Header, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(0.1)  # once in 10sec
    msg = Header()
    while not rospy.is_shutdown():
        hello_str = "Hello AWS IoT, greetings from ROS! %s" % rospy.get_time()
        msg.frame_id = hello_str
        rospy.loginfo(hello_str)
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
