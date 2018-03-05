#!/usr/bin/env python

from BathyBoatNav.srv import *
from std_srvs.srv import Trigger
from collections import defaultdict
import rospy
import json
import os.path
import sys
from functools import partial
import pyproj

global missions, nbrMissions, longitudes, latitudes, ids, nbrMissionsSent, name_mission_file, ready
ids = defaultdict(list)
latitudes = defaultdict(list)
longitudes = defaultdict(list)
missions = defaultdict(list)
nbrMissions = 0
nbrMissionsSent = 0
ready = False

def convert(latitude, longitude):

	PROJECT = partial(
	pyproj.transform,
	pyproj.Proj(init='epsg:4326'),
	pyproj.Proj(init='epsg:2154'))

	x_lambert, y_lambert = PROJECT(longitude, latitude)

	return x_lambert, y_lambert

def readFile():
	global name_mission_file

	path = "/home/helios/Helios/Missions/" + name_mission_file
	print(path)
	if os.path.isfile(path):
		with open(path, "r") as jsonFile:
			jsonDict = json.loads(jsonFile.read())
			badJson = False
			global missions, nbrMissions, longitudes, latitudes, ids
			if "missions" in jsonDict.keys():
				for i in jsonDict["missions"] :
					if all (k in i.keys() for k in ("waypoints", "type")) and i["type"] == "Waypoints" :
						missions[nbrMissions] = False;
						for j in i["waypoints"] :
							if all (k in j.keys() for k in ("lat", "lng")):
								x, y = convert(j["lat"], j["lng"])
								latitudes[nbrMissions].append(x)
								longitudes[nbrMissions].append(y)
								ids[nbrMissions].append(0)
							else:
								badJson = True
					elif all (k in i.keys() for k in ("radiales", "type")) and i["type"] == "Radiales" :
						missions[nbrMissions] = True;
						for j in i["radiales"] :
							if all (k in j.keys() for k in ("id", "start", "end")):
								start_x, start_y = convert(j["start"]["lat"], j["start"]["lng"])
								end_x, end_y = convert(j["end"]["lat"], j["end"]["lng"])

								latitudes[nbrMissions].append(start_x)
								latitudes[nbrMissions].append(end_x)
								longitudes[nbrMissions].append(start_y)
								longitudes[nbrMissions].append(end_y)
								ids[nbrMissions].append(j["id"])	
							else:
								badJson = True			
					else:
						badJson = True	
							
					nbrMissions += 1
			else:
				badJson = True	
	
			if badJson:
				rospy.logerr("Wrong conventions Json mission file")
				return False
	else:
		rospy.logerr("No mission file found")
		return False

	return True
		
def sendPointService(req):
	global missions, nbrMissions, longitudes, latitudes, nbrMissionsSent
	res = {}
	res["longitude"] = [] 
  	res["latitude"] = [] 
	if nbrMissionsSent < nbrMissions and longitudes[nbrMissionsSent]:
		res["isRadiale"] = missions[nbrMissionsSent]
		if not missions[nbrMissionsSent]:
			res["longitude"].append(longitudes[nbrMissionsSent].pop(0))
			res["latitude"].append(latitudes[nbrMissionsSent].pop(0))
		else: 
			res["longitude"].append(longitudes[nbrMissionsSent].pop(1))
			res["latitude"].append(latitudes[nbrMissionsSent].pop(1))
			res["longitude"].append(longitudes[nbrMissionsSent].pop(0))
			res["latitude"].append(latitudes[nbrMissionsSent].pop(0))
		
		if not longitudes[nbrMissionsSent]:
			nbrMissionsSent += 1
			
		res["id"] = ids[nbrMissionsSent].pop(0) 
		res["remainingMissions"] = nbrMissions - nbrMissionsSent

	return res

def newMission(req):
	global name_mission_file
	res = {}
	name_mission_file = req.message

	res['success'] = readFile()

	ready = True

	rospy.wait_for_service('changeStateSrv')
	try:
		changeStateSrv = rospy.ServiceProxy('changeStateSrv', message)
		res = changeStateSrv("PAUSE")
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

	rospy.wait_for_service('triggerAskForTarget')
	try:
		triggerAskForTarget = rospy.ServiceProxy('triggerAskForTarget', Trigger)
		res = triggerAskForTarget()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

	return res

def isReady(req):
	res['success'] = ready

	return res

if __name__ == "__main__":
	rospy.init_node('mission_interpreter', log_level=rospy.WARN)

	goal_srv 	= rospy.Service('next_goal', next_goal, sendPointService)
	mission_srv = rospy.Service('new_mission', message, newMission)
	ready_srv 	= rospy.Service('isReady', Trigger, isReady)
	rospy.spin()