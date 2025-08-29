"""
TITLE: crash_apple_082321
AUTHOR: Gemini flash 2.5 / Kai-Chun Chang
DESCRIPTION: An Apple vehicle, operating in manual mode and stopped at a stop sign in the Main Street Cupertino retail center parking lot, was struck on the left rear bumper by a diagonally-parked Subaru reversing out of a parking space at low speed. Both vehicles sustained minor damage. At the time of the incident, no injuries were reported by either party and law enforcement was not called to the scene.
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

APPLE_MODEL = "vehicle.toyota.prius"
SUBARU_MODEL = "vehicle.toyota.prius"

param SUBARU_REVERSE_THROTTLE = VerifaiRange(0.3, 0.6)
param ADVERSARY_X_OFFSET = VerifaiRange(-5, -3)
param ADVERSARY_Y_OFFSET = VerifaiRange(-6, -4)
param ADVERSARY_HEADING = VerifaiRange(310, 340) # Facing roughly EW

#################################
# AGENT BEHAVIORS               #
#################################

behavior AppleStoppedBehavior():
    while True:
        take SetBrakeAction(1)

behavior SubaruReverseBehavior():
    while True:
        take SetThrottleAction(globalParameters.SUBARU_REVERSE_THROTTLE)

#################################
# SPATIAL RELATIONS             #
#################################

egoSpawnPt = new OrientedPoint

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt, 
    facing 0 deg,
    with blueprint APPLE_MODEL,
    with behavior AppleStoppedBehavior()

adversary = new Car at ego offset by (ego.position.x + globalParameters.ADVERSARY_X_OFFSET,
                                      ego.position.y + globalParameters.ADVERSARY_Y_OFFSET),
    facing globalParameters.ADVERSARY_HEADING deg,
    with blueprint SUBARU_MODEL,
    with behavior SubaruReverseBehavior()

#################################
# REQUIREMENTS                  #
#################################

terminate when (distance to adversary) < (ego.length + adversary.length) / 2

#################################
# RECORDING                     #
#################################

record ego._boundingPolygon as egoPoly
record adversary._boundingPolygon as advPoly
