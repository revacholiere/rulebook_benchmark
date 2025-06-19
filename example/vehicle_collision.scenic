param map = localPath('../maps/Town05.xodr')
model scenic.domains.driving.model


behavior simple():
    while True:
        take SetThrottleAction(1)
        


behavior do_nothing():
    while True:
        wait

 
ego = new Car with behavior simple()
adv = new Car with behavior FollowLaneBehavior(), right of ego by 1
adv2 = new Car ahead of ego by 20, with behavior FollowLaneBehavior()



from rulebook_benchmark import bench
require monitor bench.bench()