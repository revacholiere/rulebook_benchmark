"""
TITLE: Bypassing 01
DESCRIPTION: Ego vehicle performs a lane change to bypass a slow 
adversary vehicle before returning to its original lane.
SOURCE: NHSTA, #16
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../maps/Town05.xodr')
model scenic.domains.driving.model
param POLICY = 'built_in'

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

param EGO_SPEED = VerifaiRange(7, 10)

param ADV_DIST = VerifaiRange(10, 25)
param ADV_SPEED = VerifaiRange(2, 4)

BYPASS_DIST = [15, 10]
INIT_DIST = 50
TERM_TIME = 5

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, BYPASS_DIST[0]):
        fasterLaneSec = self.laneSection.fasterLane
        do LaneChangeBehavior(
                laneSectionToSwitch=fasterLaneSec,
                target_speed=globalParameters.EGO_SPEED)
        do FollowLaneBehavior(
                target_speed=globalParameters.EGO_SPEED,
                laneToFollow=fasterLaneSec.lane) \
            until (distance to adversary) > BYPASS_DIST[1] and (apparent heading of adversary) > 1.57
        slowerLaneSec = self.laneSection.slowerLane
        do LaneChangeBehavior(
                laneSectionToSwitch=slowerLaneSec,
                target_speed=globalParameters.EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) for TERM_TIME seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

if globalParameters.POLICY == 'metadrive_ppo':
    from metadrive_expert import MetaDrivePPOPolicyCar, MetaDrivePPOPolicyBehavior, MetaDrivePPOUpdateState
    fasterLane = network.laneSectionAt(egoSpawnPt).fasterLane.lane
    nextlane = initLane.successor
    egoTrajectory = [fasterLane, nextlane]
    ego = new MetaDrivePPOPolicyCar at egoSpawnPt,
        with blueprint MODEL,
        with behavior MetaDrivePPOPolicyBehavior(egoTrajectory)
    require monitor MetaDrivePPOUpdateState()
elif globalParameters.POLICY == 'ppo_with_built_in':
    from metadrive_expert import MetaDrivePPOPolicyCar, MetaDrivePPOFollowLaneBehavior, MetaDrivePPOUpdateState
    behavior EgoPPOBehavior():
        try:
            do MetaDrivePPOFollowLaneBehavior()
        interrupt when withinDistanceToAnyObjs(self, BYPASS_DIST[0]):
            fasterLaneSec = self.laneSection.fasterLane
            do LaneChangeBehavior(
                    laneSectionToSwitch=fasterLaneSec,
                    target_speed=globalParameters.EGO_SPEED)
            do MetaDrivePPOFollowLaneBehavior() \
                until (distance to adversary) > BYPASS_DIST[1] and (apparent heading of adversary) > 1.57
            slowerLaneSec = self.laneSection.slowerLane
            do LaneChangeBehavior(
                    laneSectionToSwitch=slowerLaneSec,
                    target_speed=globalParameters.EGO_SPEED)
            do MetaDrivePPOFollowLaneBehavior() for TERM_TIME seconds
            terminate 
    ego = new MetaDrivePPOPolicyCar at egoSpawnPt,
        with blueprint MODEL,
        with behavior EgoPPOBehavior()
    require monitor MetaDrivePPOUpdateState()
else:
    ego = new Car at egoSpawnPt,
        with blueprint MODEL,
        with behavior EgoBehavior()

adversary = new Car following roadDirection for globalParameters.ADV_DIST,
    with blueprint MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require always (adversary.laneSection._fasterLane is not None)

from rulebook_benchmark import bench
require monitor bench.bench()

#################################
# RECORDING                     #
#################################

fasterLaneSec = ego.laneSection.fasterLane
record (ego in initLane or ego in initLane.successor) and (adversary can see ego) as egoReachedGoal
record ego._boundingPolygon as egoPoly
record adversary._boundingPolygon as advPoly
record ego.lane.polygon as egoLanePoly
record adversary.lane.polygon as advLanePoly