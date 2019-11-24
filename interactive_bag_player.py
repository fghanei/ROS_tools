#!/usr/bin/env python
import argparse
import rosbag
import os
import time
import rospy
import sensor_msgs.msg
import std_msgs.msg
import threading

my_topics = ["/camera/rgb/image_raw",
            "/camera/depth/image_raw"]

parser = argparse.ArgumentParser(description="Plays a bagfile interactively, as the client calls for next frames.")
parser.add_argument("-f", "--file" , help="Selecting a bag file", required=True)
args = parser.parse_args()

input_file = args.file

global jump
global initialized

jump = 1
jump_mutex = threading.Lock()

sema = threading.Semaphore(1)
initialized = False

def interact(msg):
    print("\t\t\t\t\t\t_Subscriber_ Message received with: "+str(msg.data))
    global jump
    global initialized
    jump_mutex.acquire()
    jump = msg.data
    initialized = True
    jump_mutex.release()
    sema.release()  # signaling to publish next frame
    return

rospy.init_node('interactive_player')
pub = [] #list of publishing topics rgb, and depth
pub.append(rospy.Publisher('/camera/rgb/image_raw', sensor_msgs.msg.Image, queue_size=1))
pub.append(rospy.Publisher('/camera/depth/image_raw', sensor_msgs.msg.Image, queue_size=1))
sub = rospy.Subscriber("/interactive_player/next", std_msgs.msg.Int32, interact, queue_size=10)

in_bag = rosbag.Bag(input_file)
print("------")
print("processing: " + input_file)

skip = [1, 1]  # number of frames to skip, first it's 1, then it'll interact
initial_seq = [-1, -1]  # message header sequence number of topics
current_seq = [0, 0]
covered = [False, False] # to make sure we have published both channels before waiting
published_frames = [[], []] # list of frame seqences that we publish

for topic, msg, t in in_bag.read_messages(topics=my_topics):
    if ("rgb" in topic):
        channel = 0
    else:
        channel = 1

    skip[channel] -= 1
    if (skip[channel] > 0):
        continue

    current_seq[channel] = msg.header.seq

    if (initial_seq[channel] < 0 ): # setting initial value
        initial_seq[channel] = current_seq[channel]
    current_seq[channel] -= initial_seq[channel] #taking off bias

    if (covered[channel] == False):
        pub[channel].publish(msg)
        print (topic + " - Published frame: "+str(current_seq[channel]))
        published_frames[channel].append(current_seq[channel])
        covered[channel] = True
    else: # we have already published this topic and we are going to skip this one (normaly should not happen, right?)
        pass

    if (covered[0] and covered[1]): # we have published to both channels, and should wait
        if initialized:
		sema.acquire()
	        jump_mutex.acquire()
	        skip = [jump, jump]
	        jump_mutex.release()
	        print ("Interaction received, will jump by: "+str(skip))
	else:
		print ("Not yet initialized, just sleeping and continueing")
		time.sleep(0.1)
        covered = [False, False] # reset 

sub.unregister()
in_bag.close()
print (published_frames[0])
print (published_frames[1])
print (len(published_frames[0]), " rgb frames")
print (len(published_frames[1]), " depth frames")


