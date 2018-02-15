#!/usr/bin/env python

from BathyBoatNav.srv import *
from collections import defaultdict
import rospy
import json
import os.path
import sys
from functools import partial
import pyproj

global missions, nbrMissions, longitudes, latitudes, nbrMissionsSent, name_mission_file
latitudes = defaultdict(list)
longitudes = defaultdict(list)
missions = defaultdict(list)
nbrMissions = 0
nbrMissionsSent = 0

def convert(latitude, longitude):

	PROJECT = partial(
	pyproj.transform,
	pyproj.Proj(init='epsg:4326'),
	pyproj.Proj(init='epsg:2154'))

	x_lambert, y_lambert = PROJECT(longitude, latitude)

	return x_lambert, y_lambert

def readFile():
	global name_mission_file

	path = "/home/guerledan4/BathyBoatMissions/" + name_mission_file
	print(path)
	if os.path.isfile(path):
		with open(path, "r") as jsonFile:
			jsonDict = json.loads(jsonFile.read())
			badJson = False
			global missions, nbrMissions, longitudes, latitudes
			if "missions" in jsonDict.keys():
				for i in jsonDict["missions"] :
					if all (k in i.keys() for k in ('waypoints', 'type')) and i['type'] == "Waypoints" :
						missions[nbrMissions] = False;
						for j in i['waypoints'] :
							if all (k in j.keys() for k in ('lat', 'lng')):
								x, y = convert(j['lat'], j['lng'])
								latitudes[nbrMissions].append(x)
								longitudes[nbrMissions].append(y)
							else:
								badJson = True
					elif all (k in i.keys() for k in ('radiales', 'type')) and i['type'] == "Radiales" :
						missions[nbrMissions] = True;
						for j in i['radiales'] :
							if all (k in j.keys() for k in ('start', 'end')):
								start_x, start_y = convert(j["start"]['lat'], j["start"]['lng'])
								end_x, end_y = convert(j["end"]['lat'], j["end"]['lng'])

								latitudes[nbrMissions].append(start_x)
								latitudes[nbrMissions].append(end_x)
								longitudes[nbrMissions].append(start_y)
								longitudes[nbrMissions].append(end_y)	
							else:
								badJson = True			
					else:
						badJson = True	
							
					nbrMissions += 1
			else:
				badJson = True	
	
			if badJson:
				rospy.logerr("Wrong conventions Json mission file")
				sys.exit(0)
	else:
		rospy.logerr("No mission file found")
		sys.exit(0)		
		
def sendPointService(req):
	global missions, nbrMissions, longitudes, latitudes, nbrMissionsSent
	res = {}
	res['longitude'] = [] 
  	res['latitude'] = [] 
	if nbrMissionsSent < nbrMissions and longitudes[nbrMissionsSent]:
		res['isRadiale'] = missions[nbrMissionsSent]
		if not missions[nbrMissionsSent]:
			res['longitude'].append(longitudes[nbrMissionsSent].pop(0))
			res['latitude'].append(latitudes[nbrMissionsSent].pop(0))
		else: 
			res['longitude'].append(longitudes[nbrMissionsSent].pop(1))
			res['latitude'].append(latitudes[nbrMissionsSent].pop(1))
			res['longitude'].append(longitudes[nbrMissionsSent].pop(0))
			res['latitude'].append(latitudes[nbrMissionsSent].pop(0))
			
		if not longitudes[nbrMissionsSent]:
			nbrMissionsSent += 1

		res['remainingMissions'] = nbrMissions - nbrMissionsSent

	return res
		

if __name__ == "__main__":
	rospy.init_node('mission_interpreter', log_level=rospy.WARN)

	name_mission_file = rospy.get_param('/name_mission', 'mission_rad.json')

	readFile()
	s = rospy.Service('/next_goal', next_goal, sendPointService)
	rospy.spin()
		
		

