from scenic.domains.driving.roads import Network
model scenic.domains.driving.model
from rulebook_benchmark.realization import Realization, RealizationObject, State

monitor bench():
    realization = globalParameters['realization']
    realization.network = Network.fromFile(globalParameters['map'])

    objects = simulation().objects[:-1]
    max_steps = realization.max_steps
    objs = []
    for obj in objects:
        objs.append(RealizationObject(obj.shape.mesh.copy(), obj.occupiedSpace.dimensions, type(obj).__name__))
    realization.objects = objs

    step = 0
    while True:
        objects = simulation().objects[:-1]

        for i in range(len(objects)):
            obj = realization.objects[i]
            object = objects[i]
            obj.trajectory.append(State(object.position, object.velocity, object.orientation, step))
        step += 1
        wait


