import shapely
import json
import numpy as np
from scenic.core.vectors import Vector
from cached_property import cached_property
from rulebook_benchmark.realization import State, Realization
from rulebook_benchmark.roads import OrientationVectorPlaceholder
# example trajectory data


def interpolate_between_values(start, end, delta, progress):
    unit = (end - start) / delta
    return start + unit * progress


class ReasonableCrowdObject:
    def __init__(self, dimensions, object_type):
        self.dimensions = dimensions
        self.trajectory = []
        if object_type == "ego" or object_type == "vehicle":
            self.object_type = "Car"
        elif object_type == "pedestrian":
            self.object_type = "Pedestrian"
        else:
            raise ValueError(f"Unknown object type: {object_type}")
    def get_state(self, step):
        try:
            return self.trajectory[step]
        except IndexError:
            raise Exception(f"Error: Step {step} out of bounds for object trajectory with length {len(self.trajectory)}")
        

class ReasonableCrowdState(State):
    def __init__(self, obj, position, velocity, orientation, step, footprint):
        super().__init__(obj, position, velocity, orientation, step)
        self.footprint = footprint
        self.polygon = None

    #@cached_property
    #def polygon(self):
        #return shapely.Polygon(self.footprint)


class ReasonableCrowdTrajectoryParser:
    def __init__(self, path, step_size):
        self.path = path
        self.trajectory_data = json.load(open(path, 'r'))
        self.step_size = step_size

        self._preprocess_trajectory_data()
        
    def _preprocess_trajectory_data(self):
        # aggregate each object into a list of states
        self.object_states = {}
        
        for state in self.trajectory_data:
            obj_id = state['id']
            if obj_id not in self.object_states:
                self.object_states[obj_id] = {}
            self.object_states[obj_id][state['timestamp']] = state
        
        max_end_timestamp = max(max(states.keys()) for states in self.object_states.values())


        self.object_trajectories = {}

        for obj, states in self.object_states.items():
            trajectory = []
            step = 0
            start_timestamp = min(states.keys())
            end_timestamp = max(states.keys())
            for i in range(0, max_end_timestamp+1, self.step_size):



                state = states.get(i, None)
                if state is not None:
                    pass
                elif i < start_timestamp:
                    # just use the first state
                    state = states[start_timestamp]
                elif i > end_timestamp:
                    # just use the last state
                    state = states[end_timestamp]
                else:
                    # interpolate the state
                    prev_timestamp = i - 1
                    next_timestamp = i + 1
                    while states.get(prev_timestamp, None) is None and prev_timestamp > start_timestamp:
                        prev_timestamp -= 1
                    while states.get(next_timestamp, None) is None and next_timestamp < end_timestamp:
                        next_timestamp += 1
                    prev_state = states[prev_timestamp]
                    next_state = states[next_timestamp]
                    x = interpolate_between_values(prev_state['x_meters'], next_state['x_meters'], next_timestamp - prev_timestamp, i - prev_timestamp)
                    y = interpolate_between_values(prev_state['y_meters'], next_state['y_meters'], next_timestamp - prev_timestamp, i - prev_timestamp)
                    x_velocity = interpolate_between_values(prev_state['x_velocity_meters_per_second'], next_state['x_velocity_meters_per_second'], next_timestamp - prev_timestamp, i - prev_timestamp)
                    y_velocity = interpolate_between_values(prev_state['y_velocity_meters_per_second'], next_state['y_velocity_meters_per_second'], next_timestamp - prev_timestamp, i - prev_timestamp)
                    heading = interpolate_between_values(prev_state['heading_radians'], next_state['heading_radians'], next_timestamp - prev_timestamp, i - prev_timestamp)
                    footprint = prev_state['footprint']
                    new_footprint = []
                    for j in range(len(footprint)):
                        new_footprint.append([
                            interpolate_between_values(prev_state['footprint'][j][0], next_state['footprint'][j][0], next_timestamp - prev_timestamp, i - prev_timestamp),
                            interpolate_between_values(prev_state['footprint'][j][1], next_state['footprint'][j][1], next_timestamp - prev_timestamp, i - prev_timestamp)
                        ])

                    state = {
                        'type': prev_state['type'],
                        'x_meters': x,
                        'y_meters': y,
                        'x_velocity_meters_per_second': x_velocity,
                        'y_velocity_meters_per_second': y_velocity,
                        'heading_radians': heading,
                        'timestamp': i,
                        'id': obj,
                        'footprint': new_footprint,
                    }
                state['step'] = step
                    
                step += 1
                trajectory.append(state)
            self.object_trajectories[obj] = trajectory

    def create_realization(self):
        realization = Realization()
        
        objects = []
        for obj_id, trajectory in self.object_trajectories.items():
            footprint = trajectory[0]['footprint']
            heading = trajectory[0]['heading_radians']
            previous_x = trajectory[0]['x_meters']
            previous_y = trajectory[0]['y_meters']
            
            polygon = shapely.Polygon(footprint)
            base_polygon = shapely.affinity.rotate(polygon, -heading, origin=polygon.centroid, use_radians=True)
            width = base_polygon.bounds[3] - base_polygon.bounds[1]
            length = base_polygon.bounds[2] - base_polygon.bounds[0]
            dimensions = (width, length)
            object_type = trajectory[0]['type']
            obj = ReasonableCrowdObject(dimensions, object_type)
            for state in trajectory:
                position = Vector(state['x_meters'], state['y_meters'])
                velocity = Vector(state['x_velocity_meters_per_second'], state['y_velocity_meters_per_second'])
                orientation = OrientationVectorPlaceholder(state['heading_radians'])
                current_polygon = shapely.affinity.rotate(base_polygon, state['heading_radians'], use_radians=True)
                current_polygon = shapely.affinity.translate(current_polygon, xoff=state['x_meters'] - previous_x, yoff=state['y_meters'] - previous_y)
                footprint = state['footprint']
                realization_state = ReasonableCrowdState(obj, position, velocity, orientation, state['step'], footprint)
                realization_state.polygon = current_polygon
                obj.trajectory.append(realization_state)
            objects.append(obj)
            
        realization.objects = objects
        return realization
        
                
def parse_trajectory(path, step_size):
    parser = ReasonableCrowdTrajectoryParser(path, step_size)
    return parser.create_realization()
    
            
        
