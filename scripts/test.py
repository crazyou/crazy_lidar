#!/usr/bin/env python

import math
import rospy
import os
from sensor_msgs.msg import LaserScan
import time

count = 0
dists=[[0 for col in range(120)] for row in range(400)]
var=[0 for col in range(400)]
pre_dist = 0.0

log_file_name =  "/tmp/scan_log.txt"

log_file = open(log_file_name, 'w+')

dist_log_file = open("/tmp/dist_log.txt",'w+')

def average(x, num):
	sum = 0
	for i in range(num):
		sum+=x[i]
	return sum/num

def pow(x):
	ret = x*x
	return ret
def sum(x,num):
	for i in range(num):
		sum+=x[i]
	return sum

def variance(x, num):
	sum = 0
	if num==0:
		return inf

	ave = average(x,num)
	for i in range(num):
		sum+=pow(x[i]-ave)
	sum/=num
	varia = math.sqrt(sum)
	return varia

def callback(data):
	global dist_log_file
	global count
	global dists
	global var
	global pre_dist
#	rospy.loginfo(data)
	angle_min = data.angle_min
	angle_max = data.angle_max
	
	invalid_count = 0
	for i in range(len(data.ranges)):
		if data.ranges[i]==0 or math.isinf(data.ranges[i]):
			invalid_count+=1
	valid_count = len(data.ranges)-invalid_count
	valid_percent = valid_count*1.0/len(data.ranges)*100
	print("angle min={0},angle max={1}".format(angle_min,angle_max))
	log_file.write("angle in={0}, angle max={1}\n".format(angle_min, angle_max))	
	print("dst:{0:.6},{1:.6},{2:.6},{3:.6},{4:.6},{5:.6}".format(data.ranges[0], data.ranges[1],data.ranges[2],data.ranges[3],data.ranges[4],data.ranges[5]))
	print("valid count={0}, sum={1},per={2:.3}%".format(valid_count,len(data.ranges), valid_percent))
	log_file.write("valid count={0}, sum={1}, per={2:.3}%\n".format(valid_count, len(data.ranges), valid_percent))	
	step_dist = data.ranges[0]-pre_dist
	print("curr ={0},pre={1},step = {2}".format(data.ranges[0], pre_dist, step_dist))
	print("curr_dist={0:.6},pre_dist={1:.6},step dist={2:.6}".format(data.ranges[0],pre_dist,step_dist))
	pre_dist = data.ranges[0]
	for i in range(len(data.ranges)):
		if not math.isinf(data.ranges[i]) and not data.ranges[i]==0:
			dists[i][count]=data.ranges[i]
	
	str_data = "{0},\n".format(data.ranges[0])
	dist_log_file.write(str_data)
	count = count+1
	logs = ""
	ave = 0.0
	ave0=0.0
	if count>=100:
		count = 0
		for i in range(len(data.ranges)):
			ave = average(dists[i], len(dists[i]))
			print("ave=%f"%ave)
			print("length=%d" %len(dists[i]))
			var[i]=variance(dists[i], len(dists[i]))
			var[i]=var[i]/ave
			logs += ",{0:.3}".format(var[i])
			ave0 = average(dists[0], len(dists[0]))
	print("ave0={0:.6}".format(ave0))
	rospy.loginfo(logs)
	log_file.write(logs)
	
def listener():
	
	rospy.init_node('laer_listener',anonymous=False)
	sub = rospy.Subscriber('scan',LaserScan,callback)
	rospy.spin()
	log_file.close()
	dist_log_file.close()
	

if __name__=='__main__':
	listener()




