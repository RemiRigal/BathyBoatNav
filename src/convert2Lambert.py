#!/usr/bin/env python

import rospy
from functools import partial
import pyproj

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

latitude 	= 0.0
longitude 	= 0.0
latitude_mission 	= 0.0
longitude_mission 	= 0.0

roll = pitch = yaw = 0.0

def gpsCallback(msg):
	global latitude, longitude

	latitude   = msg.latitude
	longitude  = msg.longitude
	
def missionCallback(msg):
	global latitude_mission, longitude_mission

	latitude_mission   = msg.latitude
	longitude_mission  = msg.longitude


def angleCallback(msg):
	global roll, pitch, yaw
	
	orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w];
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

def convert():
	global latitude, longitude, longitude_mission, latitude_mission

	PROJECT = partial(
	pyproj.transform,
	pyproj.Proj(init='epsg:4326'),
	pyproj.Proj(init='epsg:2154'))

	x_lambert, y_lambert = PROJECT(longitude, latitude)
	x_mission_lambert, y_mission_lambert = PROJECT(longitude_mission, latitude_mission)

	return x_lambert, y_lambert, x_mission_lambert, y_mission_lambert

rospy.init_node('convert2Lambert') 

input_GPS_msg 	= rospy.get_param('Input_GPS_msg', 'nav')
input_yaw_msg 	= rospy.get_param('Input_yaw_msg', 'imu')
output_msg 		= rospy.get_param('Output_msg', 'gps_angle_boat')
input_mission_msg = rospy.get_param('Input_mission_msg', 'mission_gps')

sub = rospy.Subscriber(input_mission_msg, NavSatFix, missionCallback)
sub = rospy.Subscriber(input_GPS_msg, NavSatFix, gpsCallback) 
sub = rospy.Subscriber(input_yaw_msg, Imu, angleCallback) 
pub = rospy.Publisher(output_msg, Twist, queue_size=10)

rate = rospy.Rate(25) 

pose = Pose2D()

while not rospy.is_shutdown(): 

	x_lambert, y_lambert, x_mission_lambert, y_mission_lambert = convert()

	pose = Twist(Vector3(x_lambert, y_lambert, yaw), Vector3(x_mission_lambert, y_mission_lambert, 0))

	pub.publish(pose) 

	rate.sleep() 
