param map = localPath('../maps/Town01.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model


behavior simple():
    while True:
        take SetThrottleAction(1)
        


behavior do_nothing():
    while True:
        wait

 
ego = new Car with behavior simple()
adv = new Car with behavior FollowLaneBehavior(), left of ego by 3
adv2 = new Car with behavior AutopilotBehavior(), ahead of ego by 20


from rulebook_benchmark import bench