param map = localPath('../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model





behavior simple():
    while True:
        take SetThrottleAction(1)
        


behavior do_nothing():
    while True:
        wait

 
ego = new Car with behavior simple()
spot = new OrientedPoint at ego offset by 5 @ 50
ped = new Pedestrian at spot, with behavior CrossingBehavior(ego, threshold=100, min_speed=1)



from rulebook_benchmark import bench
require monitor bench.bench()
