from scenic.domains.driving.roads import Network
model scenic.domains.driving.model
from rulebook_benchmark.realization import Realization, RealizationObject, State
import numpy as np

monitor bench():
    realization = globalParameters['realization']
    realization.network = Network.fromFile(globalParameters['map'])

    objects = simulation().objects
    #max_steps = realization.max_steps
    objs = []
    objs.append(RealizationObject(0 , objects[0].occupiedSpace.dimensions, type(objects[0]).__name__))  # ego

    ids = 1
    for obj in objects[1:]:
        objs.append(RealizationObject(ids, obj.occupiedSpace.dimensions, type(obj).__name__))
        ids += 1
    realization.objects = objs

    step = 0
    while True:
        objects = simulation().objects

        for i in range(len(objects)):
            obj = realization.objects[i]
            object = objects[i]
            obj.trajectory.append(State(obj, np.array([object.position.x, object.position.y]), np.array([object.velocity.x, object.velocity.y]), object.orientation, step))
        step += 1
        wait


