param map = localPath('../maps/Town01.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model


behavior simple():
    while True:
        take SetThrottleAction(1)
        


behavior do_nothing():
    while True:
        wait

 
ego = new Car with behavior FollowLaneBehavior()
spot = new OrientedPoint at ego offset by 3 @ 50
ped = new Pedestrian at spot, with behavior CrossingBehavior(ego, threshold=100, min_speed=2)


from rulebook_benchmark import bench