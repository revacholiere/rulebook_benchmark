param map = localPath('../maps/Town01.xodr')
model scenic.domains.driving.model


behavior simple():
    while True:
        take SetThrottleAction(1)
        


behavior do_nothing():
    while True:
        wait

 
ego = new Car with behavior FollowLaneBehavior()
adv = new Car with behavior FollowLaneBehavior(), left of ego by 2



from rulebook_benchmark import bench
require monitor bench.bench()