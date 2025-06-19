param map = localPath('../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.domains.driving.model





behavior simple():
    while True:
        take SetThrottleAction(1)
        



 
ego = new Car with behavior simple()
ped = new Pedestrian ahead of ego by 20



from rulebook_benchmark import bench
require monitor bench.bench()
