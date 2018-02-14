#!/usr/bin/env python

import rospy
from functools import partial
import pyproj

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import NavSatFix

global latitude, longitude, pose

latitude 	= 0
longitude 	= 0

def callback(msg): 
    latitude   = msg.latitude;
    longitude  = msg.longitude;
    print("Hey")

def convert():
	PROJECT = partial(
    pyproj.transform,
    pyproj.Proj(init='epsg:4326'),
    pyproj.Proj(init='epsg:2154'))

	x_lambert, y_lambert = PROJECT(longitude, latitude)

	return x_lambert, y_lambert

rospy.init_node('convert2Lambert') 

input_msg 	= rospy.get_param('Input_msg', 'input_msg')
output_msg 	= rospy.get_param('Output_msg', 'output_msg')


sub = rospy.Subscriber(input_msg, NavSatFix, callback) 
pub = rospy.Publisher(output_msg, Pose2D, queue_size=10)

rate = rospy.Rate(2) 

pose = Pose2D()

while not rospy.is_shutdown(): 

	x_lambert, y_lambert = convert()
	print(x_lambert, y_lambert)

	pose = Pose2D(x_lambert, y_lambert, 0)
	print(pose)

	pub.publish(pose) 

	rospy.spin()
	rate.sleep() 