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
	jsonFile = open("/home/noelie/Documents/ProjetGuerledan/Noelie/mission1.json", "r")
	jsonDict = json.loads(jsonFile.read())
	global missions, nbrMissions, longitudes, latitudes
	for i in jsonDict["missions"] :	
		if i['type'] == "WayPoint" :
			missions[nbrMissions] = False;
			for j in i['wayPoints'] :
				latitudes[nbrMissions].append(j['latitude'])
				longitudes[nbrMissions].append(j['longitude'])		
		elif i['type'] == "Radiale" :
			missions[nbrMissions] = True;
			for j in i['radiales'] :
				latitudes[nbrMissions].append(j[0]['latitude'])
				latitudes[nbrMissions].append(j[1]['latitude'])
				longitudes[nbrMissions].append(j[0]['longitude'])
				longitudes[nbrMissions].append(j[1]['longitude'])
		
		nbrMissions += 1
		
		
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
			res['longitude'].append(longitudes[nbrMissionsSent].pop(0))
			res['latitude'].append(latitudes[nbrMissionsSent].pop(0))
			
		if not longitudes[nbrMissionsSent]:
			nbrMissionsSent += 1

		res['nbrMissionNext'] = nbrMissions - nbrMissionsSent

	return res
		

if __name__ == "__main__":
	readFile()
	rospy.init_node('mission_interpreter')
	s = rospy.Service('/next_goal', next_goal, sendPointService)
	rospy.spin()
		
		

