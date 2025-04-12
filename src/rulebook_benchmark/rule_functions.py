from trimesh.transformations import compose_matrix
from scenic.core.regions import MeshVolumeRegion, EmptyRegion
import shapely
from rulebook_benchmark.realization import Realization
import math
from scenic.core.vectors import Vector

# TODO: maybe add a generalized rule loop function that takes a rule and start-end time
# TODO: shapely vs scenic distances , same for velocity/acceleration scale


class Result:
    def __init__(self, minimum_violation=0, aggregation_method=max):
        self.total_violation = minimum_violation
        self.violation_history = []
        self.aggregation_method = aggregation_method
    def add(self, violation):
        self.total_violation = self.aggregation_method((self.total_violation, violation))
        self.violation_history.append(self.total_violation)
    def __repr__(self):
        return f"Result(total_violation={self.total_violation}, history={self.violation_history})"
    def __str__(self):
        return f"Result(total_violation={self.total_violation}, history={self.violation_history})"

def rule_function(calculate_violation, aggregation_method):
    """
    A decorator to apply a rule function to a realization.
    :param calculate_violation: The function that calculates the violation.
    :param aggregate_violations: The function that aggregates the violations.
    :param realization: The realization object.
    :param start_index: The start index for the calculation.
    :param end_index: The end index for the calculation.
    :return: The total violation and the history of violations.
    """
    
    def wrapper(realization, start_index=None, end_index=None, parameters={}):
        result = Result(aggregation_method=aggregation_method)
        if start_index is None:
            start_index = 0
        
        result.violation_history += [0] * start_index
        max_steps = realization.max_steps
        
        if end_index is None:
            end_index = max_steps
            
        violation_score = 0
        for i in range(start_index, end_index):
            violation_score = calculate_violation(realization=realization, step=i, **parameters)
            result.add(violation_score)
                    
        result.violation_history += [result.total_violation] * (max_steps - end_index)
        return result
    
    return wrapper
    


    
    
def rule_collision(realization, object_type="Pedestrian", start_index=None, end_index=None):
    # re-write the function to use the new realization object
    violation_history = [0]
    
    if start_index is None:
        start_index = 0

    max_steps = realization.max_steps
    
    if end_index is None:
        end_index = max_steps
    
    ego = realization.get_ego()
    objects = [obj for obj in realization.objects_non_ego if obj.object_type == object_type]
    total_violation = 0
    
    for i in range(start_index + 1, end_index - 1):
        ego_state = ego.get_state(i)
        ego_region = MeshVolumeRegion(mesh=ego.mesh, dimensions=ego.dimensions, position=ego_state.position, rotation=ego_state.orientation)
        ego_polygon = ego_region.boundingPolygon.polygons
        ego_velocity_before = ego.get_state(i-1).velocity
        ego_velocity = ego_state.velocity
        ego_velocity_after = ego.get_state(i+1).velocity
        for obj in objects:
            obj_state = obj.get_state(i)
            obj_region = MeshVolumeRegion(mesh=obj.mesh, dimensions=obj.dimensions, position=obj_state.position, rotation=obj_state.orientation)
            obj_polygon = obj_region.boundingPolygon.polygons
            obj_velocity_before = obj.get_state(i-1).velocity
            obj_velocity = obj_state.velocity
            obj_velocity_after = obj.get_state(i+1).velocity
            if shapely.intersects(ego_polygon, obj_polygon):
                ego_delta_1 = ego_velocity - ego_velocity_before
                obj_delta_1 = obj_velocity - obj_velocity_before
                
                ego_delta_2 = ego_velocity_after - ego_velocity
                obj_delta_2 = obj_velocity_after - obj_velocity
                
                ego_delta_norm_1 = ego_delta_1.norm()
                obj_delta_norm_1 = obj_delta_1.norm()
                ego_delta_norm_2 = ego_delta_2.norm()
                obj_delta_norm_2 = obj_delta_2.norm()
                
                
                violation_score = max(ego_delta_norm_1, obj_delta_norm_1, ego_delta_norm_2, obj_delta_norm_2)
                total_violation += violation_score
        violation_history.append(total_violation)
    
    violation_history.append(total_violation)
    return total_violation, violation_history
        
def rule_vehicle_collision(realization, start_index=None, end_index=None):
    return max(rule_collision(realization, "Car", start_index=start_index, end_index=end_index), rule_collision(realization, "Truck", start_index=start_index, end_index=end_index))

def rule_vru_collision(realization, start_index=None, end_index=None):
    return max(rule_collision(realization, "Pedestrian", start_index=start_index, end_index=end_index), rule_collision(realization, "Bicycle", start_index=start_index, end_index=end_index))
    
    




# TODO: haussdorf distance does not work, use distance + intersection perhaps?
def rule_stay_in_drivable_area(realization, start_index=None, end_index=None):
    violation_history = []
    if start_index is None:
        start_index = 0

    max_steps = realization.max_steps
    
    if end_index is None:
        end_index = max_steps
        
        
    network = realization.network
    drivable_region = network.drivableRegion
    ego = realization.get_ego()
    violation_score = 0
    
    for i in range(start_index, end_index):
        state = ego.get_state(i)
        ego_region = MeshVolumeRegion(mesh=ego.mesh, dimensions=ego.dimensions, position=state.position, rotation=state.orientation)
        ego_polygon = ego_region.boundingPolygon.polygons
        drivable_polygon = drivable_region.polygons
        difference = ego_polygon.difference(drivable_polygon)
        difference_area = difference.area
        distance = shapely.distance(drivable_polygon, ego_polygon)
        violation = difference_area + distance**2
        violation_score = max(violation_score, violation)
        violation_history.append(violation_score)
        
    return violation_score, violation_history



def vru_clearance(realization, on_road=False, threshold = 2, start_index=None, end_index=None):
    violation_history = []
    
    if start_index is None:
        start_index = 0
        
    if end_index is None:
        end_index = realization.max_steps
    ego = realization.get_ego()
    objects = [obj for obj in realization.objects_non_ego if obj.object_type in ["Pedestrian", "Bicycle"]]
    drivable_region = realization.network.drivableRegion.polygons
    violation_score = 0
    
    for i in range(start_index, end_index):
        state = ego.get_state(i)
        ego_region = MeshVolumeRegion(mesh=ego.mesh, dimensions=ego.dimensions, position=state.position, rotation=state.orientation)

        for obj in objects:
            obj_state = obj.get_state(state.step)
            obj_region = MeshVolumeRegion(mesh=obj.mesh, dimensions=obj.dimensions, position=obj_state.position, rotation=obj_state.orientation)
            ego_polygon = ego_region.boundingPolygon.polygons
            obj_polygon = obj_region.boundingPolygon.polygons
            distance = ego_polygon.distance(obj_polygon)
            violation = threshold - distance
            if (on_road and shapely.intersects(drivable_region, obj_polygon)) or (not on_road and not shapely.intersects(drivable_region, obj_polygon)):
                violation_score = max(violation_score, violation)    
            violation_history.append(violation_score)
            
    return violation_score, violation_history
    



def vru_clearance_on_road(realization, start_index=None, end_index=None, threshold = 6):
    return vru_clearance(realization, on_road=True, start_index=start_index, end_index=end_index, threshold = threshold)


def vru_clearance_off_road(realization, start_index=None, end_index=None, threshold = 6):
    return vru_clearance(realization, on_road=False, start_index=start_index, end_index=end_index, threshold = threshold)


                
                
def vru_acknowledgement(realization, proximity=5, threshold=5,  timesteps=10, start_index=None, end_index=None):
    if start_index is None:
        start_index = 0
    
    if end_index is None:
        end_index = realization.max_steps
    
    
    violation_history = []
    ego = realization.get_ego()
    objects = [obj for obj in realization.objects_non_ego if obj.object_type in ["Pedestrian", "Bicycle"]]
    violation_score = 0
    
    #TODO change to dot product projection of velocity
    
    for i in range(start_index, end_index - timesteps):
        ego_state = ego.get_state(i)
        ego_future_state = ego.get_state(i+timesteps)
        ego_velocity = ego_state.velocity
        ego_next_velocity = ego_future_state.velocity
        ego_future_pos = ego_future_state.position
        for obj in objects:
            adv_current_state = obj.get_state(i)
            adv_current_pos = adv_current_state.position
            ego_future_region = MeshVolumeRegion(mesh=ego.mesh, dimensions=ego.dimensions, position=ego_future_pos, rotation=ego_future_state.orientation)
            ego_future_polygon = ego_future_region.boundingPolygon.polygons
            adv_current_region = MeshVolumeRegion(mesh=obj.mesh, dimensions=obj.dimensions, position=adv_current_pos, rotation=adv_current_state.orientation)
            adv_current_polygon = adv_current_region.boundingPolygon.polygons
            
            distance = shapely.distance(ego_future_polygon, adv_current_polygon)
            if distance < proximity:
                vec = adv_current_pos - ego_future_pos
                ego_velocity_to_adv = (ego_velocity.dot(vec)/vec.norm()**2) * vec
                ego_next_velocity_to_adv = (ego_next_velocity.dot(vec)/vec.norm()**2) * vec
                ego_velocity_to_adv_norm = ego_velocity_to_adv.norm()
                ego_next_velocity_to_adv_norm = ego_next_velocity_to_adv.norm()
                violation = threshold - (ego_velocity_to_adv_norm - ego_next_velocity_to_adv_norm)
                violation_score = max(violation_score, violation)
        violation_history.append(violation_score)
                
                
    violation_history += timesteps * [violation_score]
    return violation_score, violation_history

        
                

        

# TODO: vehicle yielding rule based on adv vehicle decelerations


def road_correct_side(realization, start_index=None, end_index=None):
    violation_history = []
    if start_index is None:
        start_index = 0

    max_steps = realization.max_steps
    
    if end_index is None:
        end_index = max_steps
        
    network = realization.network
    ego = realization.get_ego()
    violation_score = 0
    
    for i in range(start_index, end_index):
        violation = 0
        state = ego.get_state(i)
        ego_position = state.position
        ego_orientation = state.orientation
        road_orientation_field = network.roadAt(ego_position).orientation
        road_orientation = road_orientation_field.value(ego_position)
        road_yaw = road_orientation.yaw
        ego_yaw = ego_orientation.yaw
        violation = 1 if math.cos(road_yaw - ego_yaw) < 0 else 0
        violation_history.append(violation)
    violation_score = sum(violation_history)
    return violation_score, violation_history


def vehicle_clearance(realization, step, direction, threshold):
    if direction == "front":
        angle = 0
    elif direction == "left":
        angle = - math.pi / 2
    elif direction == "right":
        angle = math.pi / 2
    else:
        raise ValueError("Direction must be 'front', 'left' or 'right'")
    objects = realization.vehicles[1:]
    ego_state = realization.get_ego_state(step)
    
    ego_yaw = ego_state.orientation.yaw + angle
    ego_direction = Vector(math.cos(ego_yaw), math.sin(ego_yaw))
    ego_region = MeshVolumeRegion(mesh=realization.get_ego().mesh, dimensions=realization.get_ego().dimensions, position=ego_state.position, rotation=ego_state.orientation)
    ego_polygon = ego_region.boundingPolygon.polygons
    ego_lane = realization.network.laneAt(ego_state.position)
    if ego_lane is None:
        return 0
    violation = 0
    for obj in objects:
        obj_state = obj.get_state(step)
        obj_region = MeshVolumeRegion(mesh=obj.mesh, dimensions=obj.dimensions, position=obj_state.position, rotation=obj_state.orientation)
        obj_lane = realization.network.laneAt(obj_state.position)
        if obj_lane is None or obj_lane != ego_lane:
            continue                     
        obj_to_ego = (obj_state.position - ego_state.position)
        cosine = ego_direction.dot(obj_to_ego) / (ego_direction.norm() * obj_to_ego.norm())
        if cosine > 0:
            continue 
        obj_polygon = obj_region.boundingPolygon.polygons
        distance = ego_polygon.distance(obj_polygon)
        violation = max(violation, distance - threshold)
    return violation

            
def rule_11(realization, step, threshold=3): # front vehicle clearance
    return vehicle_clearance(realization, step, "front", threshold)
    

def rule_12(realization, step, threshold=2): # left vehicle clearance
    return vehicle_clearance(realization, step, "left", threshold)
        
        
def rule_13(realization, step, threshold=2): # right vehicle clearance
    return vehicle_clearance(realization, step, "right", threshold)

def rule_15(realization, step, threshold=10): # speed limit
    ego_velocity = realization.get_ego_state(step).velocity.norm()
    return max(0, ego_velocity - threshold)**2

def rule_18(realization, step): # lane centering
    pass



