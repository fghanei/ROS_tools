#!/bin/bash

#first input to this is the original bag file
#second input is the destination path
if [ "$#" -ne "2" ]; then
    echo "Enter input file and destination path: ./bulk.sh <input_bag> <destination>"
    exit 1
fi

for i in {1..10}
do
    ii=$(($i-1))
	my_num='1'
	for (( j=0; j<$ii; j++ ))
	do
		my_num=${my_num}0
	done
	./bag_modifier.py -f $1 -p $my_num -t 20.0 -o $2/rate_$i.bag

done
