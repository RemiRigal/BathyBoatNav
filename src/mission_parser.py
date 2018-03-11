#!/usr/bin/env python

from BathyBoatNav.srv import next_goal, message
import rospy
import json
import os


class MissionParser(object):

    def __init__(self, missionDir):
        self.missions = []
        self.missionDirectory = missionDir
        self.currentMission = 0

    def parseMissionsFromFile(self, path):
        filePath = os.path.join(self.missionDirectory, path)
        if not os.path.isfile(filePath):
            rospy.logerr('No mission file found')
            return False
        with open(filePath, 'r') as f:
            jsonMissions = json.loads(f.read())
        if not 'missions' in jsonMissions.keys():
            return False
        for jsonMission in jsonMissions['missions']:
            mission = Mission.parse(jsonMission)
            if mission is None:
                return False
            self.missions.append(mission)
        return True

    def newMission(self, req):
        self.missions = []
        self.currentMission = 0
        return { 'success': self.parseMissionsFromFile(req.message) }

    def nextGoal(self, req):
        if self.currentMission == len(self.missions):
            rospy.logerr('All missions finished')
            return None
        mission = self.missions[self.currentMission]
        point, isLast = mission.nextGoal()
        if isLast:
            self.currentMission += 1
        point['isLast'] = self.currentMission == len(self.missions)
        return point


class Mission(object):

    def __init__(self):
        self.type = None
        self.points = []
        self.currentPoint = 0

    def isRadiales(self):
        return self.type == 'Radiales'

    def isWaypoints(self):
        return self.type == 'Waypoints'

    def nextGoal(self):
        point = self.points[self.currentPoint]
        self.currentPoint += 1
        isLast = self.currentPoint == len(self.points)
        return point, isLast

    @staticmethod
    def parse(jsonMission):
        mission = Mission()
        if not 'type' in jsonMission.keys():
            return None
        mission.type = jsonMission['type']
        if (mission.isRadiales() and not 'radiales' in jsonMission.keys()) or\
            (mission.isWaypoints() and not 'waypoints' in jsonMission.keys()):
            return None
        if mission.isRadiales():
            parsed = mission.parseRadiales(jsonMission['radiales'])
            if not parsed:
                return False
        elif mission.isWaypoints():
            parsed = mission.parseWaypoints(jsonMission['waypoints'])
            if not parsed:
                return False
        else:
            return None
        return mission

    def parseRadiales(self, radiales):
        for radiale in radiales:
            keys = radiale.keys()
            if not 'start' in keys or not 'end' in keys or not 'id' in keys:
                return False
            point = {
                'latitude': [radiale['start']['lat'], radiale['end']['lat']],
                'longitude': [radiale['start']['lng'], radiale['end']['lng']],
                'id': radiale['id'],
                'isRadiales': True
            }
            self.points.append(point)
        return True

    def parseWaypoints(self, waypoints):
        for waypoint in waypoints:
            keys = waypoint.keys()
            if not 'lat' in keys or not 'lng' in keys or not 'id' in keys:
                return False
            point = {
                'latitude': [waypoint['lat']],
                'longitude': [waypoint['lng']],
                'id': waypoint['id'],
                'isRadiales': False
            }
            self.points.append(point)
        return True


if __name__ == '__main__':
    missionDirectory = rospy.get_param('/ros/missions/path')
    missionParser = MissionParser(missionDirectory)

    rospy.init_node('mission_parser')

    goalSrv = rospy.Service('next_goal', next_goal, missionParser.nextGoal)
    missionSrv = rospy.Service('new_mission', message, missionParser.newMission)

    rospy.spin()
