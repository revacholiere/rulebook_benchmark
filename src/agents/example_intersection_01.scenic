"""
TITLE: Intersection 01
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Ego vehicle goes straight at 4-way intersection and must 
suddenly stop to avoid collision when adversary vehicle from opposite 
lane makes a left turn.
SOURCE: NHSTA, #30

To run this file using the Carla simulator:
    scenic examples/carla/NHTSA_Scenarios/intersection/intersection_01.scenic --2d --model scenic.simulators.carla.model --simulate
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../maps/Town05.xodr')
model scenic.domains.driving.model
from metadrive_expert import ExpertPolicyCar, ExpertPolicyBehavior, UpdateState

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 20]
param ADV_SPEED = VerifaiRange(7, 10)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
TERM_DIST = 100

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advInitLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.reverseManeuvers)
    ).startLane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new ExpertPolicyCar at egoSpawnPt,
    with blueprint MODEL,
    with behavior ExpertPolicyBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require monitor UpdateState()
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST

#################################
# RECORDING                     #
#################################

record ego._boundingPolygon as egoPoly
record adversary._boundingPolygon as advPoly
record ego.lane.polygon as egoLanePoly
record adversary.lane.polygon as advLanePoly
record egoTrajectory[1].polygon as egoConnectingLanePoly
record egoTrajectory[2].polygon as egoEndLanePoly