#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Time

def time_keeper():
    rospy.init_node("time_keeper_node")
    pub = rospy.Publisher('used_time', Time, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    rospy.sleep(3)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        cur_time = rospy.Time.now()
        time_spent = cur_time-start_time
        pub.publish(time_spent)
        rate.sleep()

def main():
    try:
        time_keeper()
    except rospy.ROSInterruptException:
        pass

