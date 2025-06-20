param map = localPath('../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.domains.driving.model


behavior simple():
    while True:
        take SetThrottleAction(1)
        


behavior do_nothing():
    while True:
        wait

 
ego = new Car with behavior simple()
spot = new OrientedPoint at ego offset by 3 @ 30
adv = new Car with behavior FollowLaneBehavior()
ped = new Pedestrian at spot


from rulebook_benchmark import bench
require monitor bench.bench()