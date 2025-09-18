"""
TITLE: crash_waymo-august-12-2019
AUTHOR: Gemini flash 2.5 / Kai-Chun Chang
DESCRIPTION: A Waymo Autonomous Vehicle (“Waymo AV”) was in autonomous mode on northbound S. Rengstorff Avenue at Crisanto Avenue in
Mountain View when it was rear-ended. After starting to proceed following a red-to-green traffic light change, the Waymo AV yielded to a
bicyclist who merged from the bike lane into the Waymo AV's travel lane, and a passenger vehicle then made contact with the rear bumper
of the Waymo AV. The passenger vehicle was traveling at approximately 8 MPH, and the Waymo AV was traveling at approximately 3 MPH. 
The Waymo AV sustained minor damage to its rear bumper, and the passenger vehicle sustained minor damage to its front bumper.
There were no injuries reported at the scene.
SOURCE: DMV REPORT
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../maps/Town05.xodr')
model scenic.simulators.metadrive.model
from metadrive_expert import ExpertPolicyCar, ExpertPolicyBehavior, UpdateState

#################################
# CONSTANTS                     #
#################################

MODEL = "vehicle.toyota.prius"
BICYCLE_MODEL = "vehicle.bh.crossbike"

param WAYMO_SPEED = VerifaiRange(2.5, 3.5)
param WAYMO_BRAKE = VerifaiRange(0.5, 1.0)
param PASSENGER_SPEED = VerifaiRange(7, 9)
param PASSENGER_DIST = VerifaiRange(-6, -8)
param SAFETY_DIST = VerifaiRange(3, 5)
BICYCLE_SPEED = 2
BICYCLE_DIST = 10
INIT_DIST = 20
TERM_DIST = 40

#METADRIVE_ACTOR = [None]

#################################
# AGENT BEHAVIORS               #
#################################

behavior PassengerBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.PASSENGER_SPEED, trajectory=trajectory)

behavior BicycleBehavior():
    fasterLaneSec = self.laneSection.fasterLane
    do LaneChangeBehavior(laneSectionToSwitch=fasterLaneSec, target_speed=BICYCLE_SPEED)
    do FollowLaneBehavior(target_speed=BICYCLE_SPEED, laneToFollow=fasterLaneSec.lane)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new ExpertPolicyCar at egoSpawnPt,
    with blueprint MODEL,
    with behavior ExpertPolicyBehavior(egoTrajectory)

adversary = new Car following roadDirection for globalParameters.PASSENGER_DIST,
    with blueprint MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.PASSENGER_SPEED)

bicycle = new Vehicle offset by (4, BICYCLE_DIST),
    with blueprint BICYCLE_MODEL,
    with behavior BicycleBehavior()

#################################
# REQUIREMENTS                  #
#################################

require monitor UpdateState()
require (distance to intersection) < INIT_DIST
require (distance from adversary to intersection) < INIT_DIST
require bicycle.laneSection._fasterLane is not None
require next ego.lane is not bicycle.lane
#terminate when (distance to adversary) < (ego.length + adversary.length) / 2
terminate when (distance to egoSpawnPt) > TERM_DIST

#################################
# RECORDING                     #
#################################

record ego._boundingPolygon as egoPoly
record adversary._boundingPolygon as advPoly
record bicycle._boundingPolygon as bicyclePoly

record ego.lane.polygon as egoLanePoly
record adversary.lane.polygon as advLanePoly
record bicycle.lane.polygon as bicycleLanePoly
record egoTrajectory[1].polygon as egoConnectingLanePoly
record egoTrajectory[2].polygon as egoEndLanePoly

record (ego._control["throttle"]) as egoThrottle
record (ego._control["brake"]) as egoBrake
record (ego._control["steering"]) as egoSteer
