#!/usr/bin/env python
import rosbag
import os
import time
import rospy
import std_msgs.msg


rospy.init_node('interactive_client')
pub = rospy.Publisher('/interactive_player/next', std_msgs.msg.Int32, queue_size=10)

for i in range (500):
    pub.publish(i/10+1) 
    time.sleep(0.033)

