#!/usr/bin/env python

import rospy
from functools import partial
import pyproj

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

latitude 	= 0.0
longitude 	= 0.0

roll = pitch = yaw = 0.0

def gpsCallback(msg):
	global latitude, longitude

	latitude   = msg.latitude
	longitude  = msg.longitude

def angleCallback(msg):
	global roll, pitch, yaw
	
	orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w];
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

def convert():
	global latitude, longitude

	PROJECT = partial(
	pyproj.transform,
	pyproj.Proj(init='epsg:4326'),
	pyproj.Proj(init='epsg:2154'))

	x_lambert, y_lambert = PROJECT(longitude, latitude)

	return x_lambert, y_lambert

rospy.init_node('convert2Lambert') 

input_GPS_msg 	= rospy.get_param('Input_GPS_msg', 'nav')
input_yaw_msg 	= rospy.get_param('Input_yaw_msg', 'imu')
output_msg 		= rospy.get_param('Output_msg', 'gps_angle_boat')


sub = rospy.Subscriber(input_GPS_msg, NavSatFix, gpsCallback) 
sub = rospy.Subscriber(input_yaw_msg, Imu, angleCallback) 
pub = rospy.Publisher(output_msg, Pose2D, queue_size=10)

rate = rospy.Rate(25) 

pose = Pose2D()

while not rospy.is_shutdown(): 

	x_lambert, y_lambert = convert()

	pose = Pose2D(x_lambert, y_lambert, yaw)

	pub.publish(pose) 

	rate.sleep() 
