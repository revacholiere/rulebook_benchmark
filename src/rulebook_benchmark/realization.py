# TODO: change naming of class methods
from cached_property import cached_property
from scenic.core.regions import MeshVolumeRegion

class Realization():
    def __init__(self, ego_index=0, delta=0.1):
        self.network = None
        self.objects = None
        self.ego_index = ego_index
        self.delta = delta

    def __len__(self):
        return len(self.objects[self.ego_index].trajectory)

    @property
    def ego(self):
        if not isinstance(self.ego_index, int):
            raise Exception(f"Error: Ego index {self.ego_index} is not an integer")
        if self.ego_index < 0 or self.ego_index >= len(self.objects):
            raise Exception(f"Error: Ego index {self.ego_index} out of bounds for objects list of length {len(self.objects)}")
        return self.objects[self.ego_index]

    def get_ego(self):
        try:
            return self.objects[self.ego_index]
        except IndexError:
            raise Exception(f"Error: Ego index {self.ego_index} not found in objects list")

    def get_world_state(self, step):
        states = []
        for i in range(len(self.objects)):
            states.append(self.objects[i].get_state(step))
        return WorldState(states, step, self.ego_index)

    def set_ego_index(self, ego_index):
        if 0 <= ego_index < len(self.objects):
            self.ego_index = ego_index
        else:
            raise Exception(f"Error: Proposed ego index {ego_index} out of list bounds")

    def get_ego_index(self):
        return self.ego_index

    @property
    def trajectory(self):
        trajectory = []
        for i in range(self.max_steps):
            states = self.get_world_state(i)
            trajectory.append(states)
        return trajectory

    @property
    def objects_non_ego(self):
        return self.objects[:self.ego_index] + self.objects[self.ego_index + 1:]
    
    @cached_property
    def vehicles(self):
        vehicles = []
        for obj in self.objects:
            if obj.object_type == "Car" or obj.object_type == "Truck":
                vehicles.append(obj)
        return vehicles
    
    @property
    def vehicles_non_ego(self):
        return self.vehicles[:self.ego_index] + self.vehicles[self.ego_index + 1:]
    
    @cached_property
    def VRUs(self):
        VRUs = []
        for obj in self.objects:
            if obj.object_type == "Pedestrian" or obj.object_type == "Bicycle":
                VRUs.append(obj)
        return VRUs
    


class RealizationObject():
    def __init__(self, mesh, dimensions, object_type):
        self.mesh = mesh
        self.dimensions = dimensions
        self.object_type = object_type
        self.trajectory = []

    def get_state(self, step):
        try:
            return self.trajectory[step]
        except IndexError:
            raise Exception(f"Error: Step {step} not found in object trajectory")
        
    def get_polygon(self, state):
        return MeshVolumeRegion(mesh=self.mesh, position=state.position, rotation=state.orientation, dimensions=self.dimensions).boundingPolygon.polygons
        


class State():
    def __init__(self, obj, position, velocity, orientation, step):
        self.object = obj
        self.position = position
        self.velocity = velocity
        self.orientation = orientation
        self.step = step
        self.lane = None # to be set in process_trajectory
        
    @property
    def orientation_trimesh(self):
        return self.orientation._trimeshEulerAngles()
    @cached_property
    def polygon(self):
        return self.object.get_polygon(self)
    



class WorldState():
    def __init__(self, states, step, ego_index):
        self.ego_index = ego_index
        self.states = states
        self.step = step
        
    def __getitem__(self, index):
        if index < 0 or index >= len(self.states):
            raise IndexError(f"Index {index} out of bounds for states list")
        return self.states[index]

    def get_ego_state(self):
        return self[self.ego_index]

    @property
    def other_states(self):
        return self.states[:self.ego_index] + self.states[self.ego_index + 1:]
    @property
    def other_vehicle_states(self):
        return [state for state in self.other_states if state.object.object_type in ["Car", "Truck"]]
    @property
    def ego_state(self):
        return self.get_ego_state()
    @property
    def vru_states(self):
        return [state for state in self.states if state.object.object_type in ["Pedestrian", "Bicycle"]]
    
    

