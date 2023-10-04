#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Header


def talker():
    pub = rospy.Publisher("chatter", Header, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(0.5)  # once in 2sec
    msg = Header()
    seq_cnt = 0
    while not rospy.is_shutdown():
        hello_str = "Hello AWS IoT, greetings from ROS!"
        msg.frame_id = hello_str
        msg.stamp = rospy.get_rostime()
        msg.seq = seq_cnt
        rospy.loginfo(hello_str)
        pub.publish(msg)
        rate.sleep()
        seq_cnt += 1


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
