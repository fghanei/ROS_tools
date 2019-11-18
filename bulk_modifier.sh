#!/bin/bash

for i in {61..80}
do
    ii=$(($i-1))
	my_num='1'
	for (( j=0; j<$ii; j++ ))
	do
		my_num=${my_num}0
	done
	./bag_modifier.py -f rate_1.bag -p $my_num -o rate_$i.bag

done
