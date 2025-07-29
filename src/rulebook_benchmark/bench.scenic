from scenic.domains.driving.roads import Network
model scenic.domains.driving.model
from rulebook_benchmark.realization import Realization, RealizationObject, State

monitor bench():
    realization = globalParameters['realization']
    realization.network = Network.fromFile(globalParameters['map'])

    objects = simulation().objects
    #max_steps = realization.max_steps
    objs = []
    for obj in objects:
        objs.append(RealizationObject(obj.shape.mesh.copy(), obj.occupiedSpace.dimensions, type(obj).__name__))
    realization.objects = objs

    step = 0
    while True:
        objects = simulation().objects

        for i in range(len(objects)):
            obj = realization.objects[i]
            object = objects[i]
            obj.trajectory.append(State(obj, object.position, object.velocity, object.orientation, step, object.steer, object.throttle, object.brake))
        step += 1
        wait


