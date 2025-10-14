param map = localPath('../../maps/Town05.xodr')
model scenic.domains.driving.model

#param EGO_SPEED = VerifaiRange(7, 10)
#param ADV_SPEED = VerifaiRange(7, 10)

scenario LeftTurnAdv(intersection, gap=0.5):
    setup:
        advInitLane = ego.lane #Uniform(*intersection.incomingLanes)
        advSpawnPt = new OrientedPoint ahead of ego by gap
        advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers))
        advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
        adv = new Car at advSpawnPt, 
            with behavior FollowTrajectoryBehavior(target_speed=5, trajectory=advTrajectory)
        #adv = new Car behind ego by 0.5, with behavior FollowLaneBehavior(target_speed=5)

scenario RightTurnAdv(intersection, gap=0.5):
    setup:
        advInitLane = ego.lane #Uniform(*intersection.incomingLanes)
        advSpawnPt = new OrientedPoint ahead of ego by gap
        advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, advInitLane.maneuvers))
        advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
        adv = new Car at advSpawnPt, 
            with behavior FollowTrajectoryBehavior(target_speed=5, trajectory=advTrajectory)
        #adv = new Car behind ego by 0.5, with behavior FollowLaneBehavior(target_speed=5)

scenario StraightEgo(intersection):
    setup:
        #print('incoming lanes:', intersection.incomingLanes)
        egoInitLane = Uniform(*intersection.incomingLanes)
        egoSpawnPt = new OrientedPoint in egoInitLane.centerline
        egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
        egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
        ego = new Car at egoSpawnPt,
            with behavior FollowTrajectoryBehavior(target_speed=5, trajectory=egoTrajectory)
        #print('lane:', egoInitLane.uid, ego.lane.uid)

scenario Main():
    setup:
        intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

        egoScenario = StraightEgo(intersection)
        advScenario = LeftTurnAdv(intersection, gap=3)
        advScenario2 = LeftTurnAdv(intersection, gap=-10)

        record egoScenario.ego._boundingPolygon as egoPoly
        record egoScenario.ego.lane.polygon as egoLanePoly
        record advScenario.adv._boundingPolygon as advPoly
        record advScenario.adv.lane.polygon as advLanePoly
        record advScenario2.adv._boundingPolygon as adv2Poly
        record advScenario2.adv.lane.polygon as adv2LanePoly
    compose:
        do egoScenario, advScenario, advScenario2
        #while True:
        #    do egoScenario, advScenario

Main()