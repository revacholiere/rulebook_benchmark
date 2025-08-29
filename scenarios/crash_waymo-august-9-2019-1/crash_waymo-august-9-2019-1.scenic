"""
TITLE: crash_waymo-august-9-2019-1
AUTHOR: Gemini flash 2.5 / Kai-Chun Chang
DESCRIPTION: A Waymo Autonomous Vehicle (“Waymo AV”) was in autonomous mode on the off-ramp from eastbound Alma Street to southbound Oregon Expressway in Palo Alto when it was rear-ended. The Waymo AV was traveling approximately less than 1 MPH and yielding to cross-traffic when a passenger vehicle made contact with the Waymo AV's rear bumper at approximately 5 MPH. The Waymo AV sustained minor damage to the rear bumper, and the passenger vehicle sustained minor damage to its front bumper. There were no injuries reported at the scene.
SOURCE: DMV REPORT
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../maps/Town05.xodr')
model scenic.domains.driving.model

#################################
# CONSTANTS                     #
#################################

MODEL = "vehicle.toyota.prius"

param WAYMO_SPEED = VerifaiRange(0.5, 0.9)  # Waymo AV traveling less than 1 MPH
param PASSENGER_SPEED = VerifaiRange(4.5, 5.5) # Passenger vehicle traveling approximately 5 MPH
param PASSENGER_DIST = VerifaiRange(-6, -9) # Initial distance of passenger vehicle behind Waymo AV
INIT_DIST = 20
TERM_DIST = 30

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaymoBehavior(trajectory):
    # The Waymo AV is already yielding and traveling at a low speed.
    do FollowTrajectoryBehavior(target_speed=globalParameters.WAYMO_SPEED, trajectory=trajectory)

behavior PassengerBehavior():
    # The passenger vehicle follows its current lane at the specified speed.
    do FollowLaneBehavior(target_speed=globalParameters.PASSENGER_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable intersection for the off-ramp scenario.
# A generic 4-way intersection is used for flexibility.
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# The Waymo AV is on an off-ramp, which implies a turn maneuver.
# We model this as a right turn from an incoming lane.
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# The Waymo AV (ego) is placed at the spawn point and executes its behavior.
ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior WaymoBehavior(egoTrajectory)

# The passenger vehicle (adversary) is spawned behind the Waymo AV
# and follows its lane, leading to a rear-end collision.
adversary = new Car following roadDirection for globalParameters.PASSENGER_DIST,
    with blueprint MODEL,
    with behavior PassengerBehavior()

#################################
# REQUIREMENTS                  #
#################################

# Both vehicles should start near the chosen intersection.
require (distance to intersection) < INIT_DIST
require (distance from adversary to intersection) < INIT_DIST

# The scenario terminates upon contact between the Waymo AV and the passenger vehicle.
terminate when (distance to adversary) < (ego.length + adversary.length) / 2 + 0.01
# Secondary termination condition to prevent infinite simulations.
terminate when (distance to egoSpawnPt) > TERM_DIST

#################################
# RECORDING                     #
#################################

record ego._boundingPolygon as egoPoly
record adversary._boundingPolygon as advPoly
