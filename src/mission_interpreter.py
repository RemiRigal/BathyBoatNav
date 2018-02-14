#!/usr/bin/env python

from BathyBoatNav.srv import *
from collections import defaultdict
import rospy
import json
import os.path
import sys

global missions, nbrMissions, longitudes, latitudes, nbrMissionsSent
latitudes = defaultdict(list)
longitudes = defaultdict(list)
missions = defaultdict(list)
nbrMissions = 0
nbrMissionsSent = 0

def readFile():
	path = "/home/guerledan4/BathyBoatMissions/mission.json"
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
								latitudes[nbrMissions].append(j['lat'])
								longitudes[nbrMissions].append(j['lng'])
							else:
								badJson = True
					elif all (k in i.keys() for k in ('radiales', 'type')) and i['type'] == "Radiales" :
						missions[nbrMissions] = True;
						for j in i['radiales'] :
							if all (k in j.keys() for k in ('start', 'end')):
								latitudes[nbrMissions].append(j["start"]['lat'])
								latitudes[nbrMissions].append(j["end"]['lat'])
								longitudes[nbrMissions].append(j["start"]['lng'])
								longitudes[nbrMissions].append(j["end"]['lng'])	
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
	readFile()
	s = rospy.Service('/next_goal', next_goal, sendPointService)
	rospy.spin()
		
		

