#!/usr/bin/env python

from BathyBoatNav.srv import *
from collections import defaultdict
import rospy
import json

global missions, nbrMissions, longitudes, latitudes, nbrMissionsSent
latitudes = defaultdict(list)
longitudes = defaultdict(list)
missions = defaultdict(list)
nbrMissions = 0
nbrMissionsSent = 0

def readFile():
	jsonFile = open("/home/lallemfa/BathyBoatMissions/mission1.json", "r")
	jsonDict = json.loads(jsonFile.read())
	global missions, nbrMissions, longitudes, latitudes
	for i in jsonDict["missions"] :	
		if i['type'] == "Waypoints" :
			missions[nbrMissions] = False;
			for j in i['waypoints'] :
				latitudes[nbrMissions].append(j['lat'])
				longitudes[nbrMissions].append(j['lng'])		
		elif i['type'] == "Radiales" :
			missions[nbrMissions] = True;
			for j in i['radiales'] :
				latitudes[nbrMissions].append(j[0]['lat'])
				latitudes[nbrMissions].append(j[1]['lat'])
				longitudes[nbrMissions].append(j[0]['lng'])
				longitudes[nbrMissions].append(j[1]['lng'])
		
		nbrMissions += 1

def sendPointService(req):
	global missions, nbrMissions, longitudes, latitudes, nbrMissionsSent
	res = {}
	if nbrMissionsSent < nbrMissions and longitudes[nbrMissionsSent]:
		res['isRadiale'] = missions[nbrMissionsSent]
		if not missions[nbrMissionsSent]:

			res['longitude'] = longitudes[nbrMissionsSent].pop(0)
			res['latitude'] = latitudes[nbrMissionsSent].pop(0)
		else: 
			res['longitude'] = longitudes[nbrMissionsSent].pop(0)
			res['latitude'] = latitudes[nbrMissionsSent].pop(0)
			
		if not longitudes[nbrMissionsSent]:
			nbrMissionsSent += 1

		res['remainingMissions'] = nbrMissions - nbrMissionsSent

	return res
		

if __name__ == "__main__":
	readFile()
	rospy.init_node('mission_interpreter')
	s = rospy.Service('/next_goal', next_goal, sendPointService)
	rospy.spin()
		
		

