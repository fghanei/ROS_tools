#!/usr/bin/env python
import rosbag
import os
import time
import rospy
import std_msgs.msg
import std_srvs.srv


rospy.init_node('interactive_client')
pub = rospy.Publisher('/interactive_player/next', std_msgs.msg.Int32, queue_size=1)

print("Waiting for the service advertisement")
rospy.wait_for_service("/interactive_player/start") #optional
print("Waiting a bit to stablize...")
time.sleep(1)
print("Defining ServiceProxy...")
start_trigger = rospy.ServiceProxy("/interactive_player/start", std_srvs.srv.Empty)
print("Calling the trigger function...")
try:
    start_trigger()
except:
    print("An exception has occured, not sure why!")
print("Entering the loop...")
for i in range (700):
    pub.publish(1) 
    time.sleep(0.033)

