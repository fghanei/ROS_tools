#!/usr/bin/env python
import argparse
import rosbag
import os

my_topics = ["/cv_camera/image_raw",
            "/camera/rgb/image_raw",
            "/camera/depth/image_raw"]

parser = argparse.ArgumentParser(description="Modify a bagfile to desired time scale, and frame pattern. For now the topics are hardcoded.")
parser.add_argument("-f", "--file" , help="Selecting a single file (Overrides -d)")
parser.add_argument("-d", "--directory" , help="The directory to process bag files (Default = .)")
parser.add_argument("-t", "--timescale",  type=float, help="The scale to which each frame will be stretched  so 1 second becomes T seconds (Default=1.0)")
parser.add_argument("-o", "--output",  help="Output file name  (Default=result.bag")
parser.add_argument("-p", "--pattern",  help="Pattern at which frames will be picked or dropped, indicated with 1 (pick) and 0 (drop) e.g. 10 or 1001000 (Default=1)")
args = parser.parse_args()

pattern_string = "1"  # Pick True/False for each set you want to pick. for exapmle TF means every other message. TFF means one out of three. TTFF, TFFTFFF, etc.

input_directory = "." # files within ths directory will be picked and processed. you can manually modify input_bags if needed.
SINGLE_FILE = False

output_name = "result.bag"

time_scale = 1.0 #stretch the timescale by this (so 1.0 second will become time_scale seconds)


if args.directory:
    input_directory = args.directory
    if (input_directory.endswith('/')):
            input_directory = input_directory[:-1]
if args.timescale:
    time_scale=float(args.timescale)
if args.output:
    output_name = args.output
if args.pattern:
    pattern_string = args.pattern
if args.file:
    input_bags = [args.file]
    SINGLE_FILE = True

print "-------------------------------------"
print "time_scale = ",time_scale
print "output file = ", output_name
print "selected pattern = ", pattern_string
if (SINGLE_FILE):
    print "single input file selected = ", input_bags[0]
    if (input_bags[0] == output_name):
        print "*******************************************************************"
        print "*** input file and output file have the same names, Aborting... ***"
        print "*******************************************************************"
        exit(0)

else:
    print "looking for input files in: " , input_directory
    input_bags=os.listdir(input_directory)
    if (output_name) in input_bags:
        print "***********************************************************************"
        print "*** File with the output_name exists among input files, Aborting... ***"
        print "***********************************************************************"
        exit(0)
    input_bags.sort() 
    input_bags = [input_directory+"/"+filename for filename in input_bags if filename.endswith(".bag")]
    print "input files: "
    print input_bags


initial_seq = {}
initial_timestamp = 0

for topic in my_topics:
    initial_seq.update({topic : -1})

out_bag = rosbag.Bag(output_name, 'w')

for current_bag in input_bags:
    in_bag = rosbag.Bag(current_bag)
    print "processing: " + current_bag

    for topic, msg, t in in_bag.read_messages(topics=my_topics):
        current_seq = msg.header.seq
        if (initial_seq[topic] < 0): #first message on this topic
            initial_seq[topic] = current_seq
            if (initial_timestamp == 0):
                initial_timestamp = t
#                print "initial time is: ", t

        pattern_offset = (current_seq - initial_seq[topic])%len(pattern_string)
        if (pattern_string[pattern_offset]=='1'):
            out_bag.write(topic, msg, initial_timestamp + time_scale * (t - initial_timestamp))
    in_bag.close()

out_bag.close()
