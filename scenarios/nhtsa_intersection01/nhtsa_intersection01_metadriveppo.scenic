"""
TITLE: Intersection 01
DESCRIPTION: Ego vehicle goes straight at 4-way intersection and must 
suddenly stop to avoid collision when adversary vehicle from opposite 
lane makes a left turn.
SOURCE: NHSTA, #30
POLICY: MetaDrive PPO Agent
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../maps/Town05.xodr')
model scenic.domains.driving.model
from metadrive_expert import MetaDrivePPOPolicyCar, MetaDrivePPOPolicyBehavior, MetaDrivePPOUpdateState

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
TERM_DIST = 70

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

ego = new MetaDrivePPOPolicyCar at egoSpawnPt,
    with blueprint MODEL,
    with behavior MetaDrivePPOPolicyBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST

from rulebook_benchmark import bench
require monitor bench.bench()
require monitor MetaDrivePPOUpdateState()

#################################
# RECORDING                     #
#################################

record ego in egoManeuver.endLane as egoReachedGoal
record ego._boundingPolygon as egoPoly
record adversary._boundingPolygon as advPoly
record ego.lane.polygon as egoLanePoly
record adversary.lane.polygon as advLanePoly
record egoTrajectory[1].polygon as egoConnectingLanePoly
record egoTrajectory[2].polygon as egoEndLanePoly