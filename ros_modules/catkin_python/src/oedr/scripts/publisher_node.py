#!/usr/bin/env

import rospy
from std_msgs.msg import String
from oedr.msg import Position


def talk_to_me():
    
    pub = rospy.Publisher('talking_topic', Position, queue_size=10)
    # initialize node
    rospy.init_node('publisher_node', anonymous=True)
    rate = rospy.Rate(1)
    # log
    rospy.loginfo('Publisher Node is now publishing messages')
    # while node is running
    while not rospy.is_shutdown():
        msg = Position()
        msg.message = "My position is :"
        msg.x = 2.2
        msg.y = 1.5
        pub.publish(msg)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        talk_to_me()
    except rospy.ROSInterruptException:
        pass