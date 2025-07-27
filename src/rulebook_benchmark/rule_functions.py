from trimesh.transformations import compose_matrix
from scenic.core.regions import MeshVolumeRegion, EmptyRegion
import shapely
from rulebook_benchmark.realization import Realization, State, RealizationObject
import math
from scenic.core.vectors import Vector
from rulebook_benchmark.rulebook import Rule
import numpy as np

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
    
    def wrapper(realization, start_index=None, end_index=None, **parameters):
        result = Result(aggregation_method=aggregation_method)
        if start_index is None:
            start_index = 0
        
        result.violation_history += [0] * start_index
        max_steps = len(realization.ego.trajectory) - 1
        #max_steps = realization.max_steps
        
        if end_index is None:
            end_index = max_steps
            
        violation_score = 0
        for i in range(start_index, end_index + 1):
            violation_score = calculate_violation(realization, i, **parameters)
            #violation_score, carryover = calculate_violation(realization, i, carryover=carryover, **parameters)
            result.add(violation_score)
                    
        result.violation_history += [result.total_violation] * (max_steps - end_index)
        return result
    
    return wrapper

"""  
def rule_collision(realization, object_type="Pedestrian", start_index=None, end_index=None):
    # re-write the function to use the new realization object
    violation_history = [0]
    
    if start_index is None:
        start_index = 0

    max_steps = realization.max_steps
    
    if end_index is None:
        end_index = max_steps
    
    ego = realization.ego
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
"""

def rule_vru_collision(realization, step, car_mass=1500, vru_mass=70): # looks good, but the normalization can be done with Scenic's built-in functions? also, maybe the timestep right before collision could be a starting point?
    # also, we could unify the VRU and vehicle rules and add a parameter?
    ego = realization.ego
    ego_state = ego.get_state(step)
    ego_polygon = ego_state.polygon
    ego_velocity = (ego_state.velocity[0], ego_state.velocity[1])
    
    objects = [obj for obj in realization.objects_non_ego if obj.object_type in ["Pedestrian", "Bicycle"]]
    violation = 0
    
    for obj in objects:
        obj_state = obj.get_state(step)
        obj_polygon = obj_state.polygon
        obj_velocity = (obj_state.velocity[0], obj_state.velocity[1])
        if not ego_polygon.intersects(obj_polygon):
            continue
        if step > 0:
            if shapely.intersects(ego.get_state(step - 1).polygon, obj.get_state(step - 1).polygon):
                continue
        # Collision starts at this step
        for i in range(step + 1, len(ego.trajectory)):
            next_ego_state = ego.get_state(i)
            next_obj_state = obj.get_state(i)
            next_ego_polygon = next_ego_state.polygon
            next_obj_polygon = next_obj_state.polygon
            if shapely.intersects(next_ego_polygon, next_obj_polygon) and i < len(ego.trajectory) - 1:
                # Collision continues
                continue
            # Collision ends at the i-th step
            ego_after_velocity = (next_ego_state.velocity[0], next_ego_state.velocity[1])
            obj_after_velocity = (next_obj_state.velocity[0], next_obj_state.velocity[1])
            current_violation = 0.5 * car_mass * (np.linalg.norm(ego_velocity) ** 2 - np.linalg.norm(ego_after_velocity) ** 2) + \
                                0.5 * vru_mass * (np.linalg.norm(obj_after_velocity) ** 2 - np.linalg.norm(obj_velocity) ** 2)
            violation = max(violation, current_violation)
            break
    
    return violation

def rule_vehicle_collision(realization, step, car_mass=1500):
    ego = realization.ego
    ego_state = ego.get_state(step)
    ego_polygon = ego_state.polygon
    ego_velocity = (ego_state.velocity[0], ego_state.velocity[1])
    
    objects = [obj for obj in realization.objects_non_ego if obj.object_type in ["Car", "Truck"]]
    violation = 0
    
    for obj in objects:
        obj_state = obj.get_state(step)
        obj_polygon = obj_state.polygon
        obj_velocity = (obj_state.velocity[0], obj_state.velocity[1])
        if not ego_polygon.intersects(obj_polygon):
            continue
        if step > 0:
            if shapely.intersects(ego.get_state(step - 1).polygon, obj.get_state(step - 1).polygon):
                continue
        # Collision starts at this step
        for i in range(step + 1, len(ego.trajectory)):
            next_ego_state = ego.get_state(i)
            next_obj_state = obj.get_state(i)
            next_ego_polygon = next_ego_state.polygon
            next_obj_polygon = next_obj_state.polygon
            if shapely.intersects(next_ego_polygon, next_obj_polygon) and i < len(ego.trajectory) - 1:
                # Collision continues
                continue
            # Collision ends at the i-th step
            ego_after_velocity = (next_ego_state.velocity[0], next_ego_state.velocity[1])
            obj_after_velocity = (next_obj_state.velocity[0], next_obj_state.velocity[1])
            current_violation = 0.5 * car_mass * (np.linalg.norm(ego_velocity) ** 2 - np.linalg.norm(ego_after_velocity) ** 2) + \
                                0.5 * car_mass * (np.linalg.norm(obj_velocity) ** 2 - np.linalg.norm(obj_after_velocity) ** 2)
            violation = max(violation, current_violation)
            break
    
    return violation

def collision_modified(realization, step, object_type="VRU", car_mass=1500, vru_mass=70):
    if object_type == "Vehicle":
        mass = car_mass
        objects = realization.VRUs
    elif object_type == "VRU":
        mass = vru_mass
        objects = realization.vehicles_non_ego
    else:
        raise ValueError(f"Invalid object type: {object_type}. Must be 'Vehicle' or 'VRU'.")
    
    ego = realization.ego
    violation = 0
    
    for obj in objects:
        obj_state = obj.get_state(step)
        obj_polygon = obj_state.polygon
        ego_state = ego.get_state(step)
        ego_polygon = ego_state.polygon
        
        if not ego_polygon.intersects(obj_polygon):
            continue
        
        if step > 0:
            if shapely.intersects(ego.get_state(step - 1).polygon, obj.get_state(step - 1).polygon):
                continue
        
        ego_before = ego.get_state(step-1) # state before collision
        obj_before = obj.get_state(step-1)
        
        kinetic_energy_before = 0.5 * mass * (ego_before.velocity.norm()) + 0.5 * mass * (obj_before.velocity.norm())
        
        # Collision starts at this step
        for i in range(step + 1, len(ego.trajectory) - 1):
            next_ego_state = ego.get_state(i)
            next_obj_state = obj.get_state(i)
            next_ego_polygon = next_ego_state.polygon
            next_obj_polygon = next_obj_state.polygon
            
            if shapely.intersects(next_ego_polygon, next_obj_polygon) and i < len(ego.trajectory) - 1:
                # Collision continues
                continue
            
            # Collision ends at the i-th step
            kinetic_energy_after = 0.5 * mass * (next_ego_state.velocity.norm() ** 2) + 0.5 * mass * (next_obj_state.velocity.norm() ** 2)
            violation = max(violation, kinetic_energy_before - kinetic_energy_after)
            
    return violation
            

def vru_collision(realization, step, **kwargs):
    return collision_modified(realization, step, object_type="VRU", **kwargs)    

f1 = rule_function(rule_vru_collision, max)             

def vehicle_collision(realization, step, **kwargs):
    return collision_modified(realization, step, object_type="Vehicle", **kwargs)

f2 = rule_function(rule_vehicle_collision, max)

def vru_time_to_collision(realization, step, threshold=1.0, step_size=0.04):
    
    def check_intersection(vehicle_polygon, pedestrian_polygon, velocity, step_size=step_size, threshold_time=threshold):
        vx, vy = velocity
        num_steps = int(threshold_time / step_size)
        for step in range(num_steps):
            current_time = step * step_size
            dx = vx * step_size
            dy = vy * step_size
            vehicle_polygon = shapely.affinity.translate(vehicle_polygon, xoff=dx, yoff=dy)
            if vehicle_polygon.intersects(pedestrian_polygon):
                return True, current_time
            
        return False, threshold_time
    
    ego = realization.ego
    ego_state = ego.get_state(step)
    ego_polygon = ego_state.polygon
    ego_velocity = (ego_state.velocity[0], ego_state.velocity[1])
    
    objects = [obj for obj in realization.objects_non_ego]
    violation = 0
    
    for obj in objects:
        if obj.object_type not in ["Pedestrian", "Bicycle"]:
            continue
        obj_state = obj.get_state(step)
        obj_polygon = obj_state.polygon
        to_collide, ttc = check_intersection(ego_polygon, obj_polygon, ego_velocity, step_size=step_size, threshold_time=threshold)
        if to_collide:
            assert ttc <= threshold, f"TTC {ttc} exceeds threshold {threshold}"
            violation = max(violation, threshold - ttc)
        
    return violation

f4 = rule_function(vru_time_to_collision, max)

def vehicle_time_to_collision(realization, step, threshold=1.0, step_size=0.05):
    
    def check_intersection(ego_polygon, adv_polygon, ego_velocity, adv_velocity, step_size=step_size, threshold_time=threshold):
        ego_vx, ego_vy = ego_velocity # this may get "too many values to unpack" error, see test.ipynb code blocks 3, 4, 5
        adv_vx, adv_vy = adv_velocity
        num_steps = int(threshold_time / step_size)
        min_dist = float("inf")
        for step in range(num_steps):
            current_time = step * step_size
            dx = ego_vx * step_size
            dy = ego_vy * step_size
            ego_polygon = shapely.affinity.translate(ego_polygon, xoff=dx, yoff=dy)
            adv_dx = adv_vx * step_size
            adv_dy = adv_vy * step_size
            adv_polygon = shapely.affinity.translate(adv_polygon, xoff=adv_dx, yoff=adv_dy)
            min_dist = min(min_dist, shapely.distance(ego_polygon, adv_polygon))
            if ego_polygon.intersects(adv_polygon):
                return True, current_time
        return False, threshold_time
    
    def check_orientation(ego_position, adv_position, ego_velocity): # what to do in the U-turn example?
        """
        Check if the ego vehicle is moving towards the other vehicle.
        """
        relative_position = (adv_position[0] - ego_position[0], adv_position[1] - ego_position[1]) 
        if np.dot(relative_position, ego_velocity) < 0: # does this confirm that the ego vehicle is moving towards the other vehicle?
            return False
        return True
    
    ego = realization.ego
    ego_state = ego.get_state(step)
    ego_polygon = ego_state.polygon
    ego_position = (ego_state.position[0], ego_state.position[1])
    ego_velocity = (ego_state.velocity[0], ego_state.velocity[1])
    
    objects = [obj for obj in realization.objects_non_ego]
    violation = 0
    
    for obj in objects:
        if obj.object_type not in ["Car", "Truck"]:
            continue
        obj_state = obj.get_state(step)
        obj_polygon = obj_state.polygon
        obj_position = (obj_state.position[0], obj_state.position[1])
        obj_velocity = (obj_state.velocity[0], obj_state.velocity[1])
        if not check_orientation(ego_position, obj_position, ego_velocity):
            continue
        to_collide, ttc = check_intersection(ego_polygon, obj_polygon, ego_velocity, obj_velocity, step_size=step_size, threshold_time=threshold)
        if to_collide:
            assert ttc <= threshold, f"TTC {ttc} exceeds threshold {threshold}"
            violation = max(violation, threshold - ttc)
       
    return violation
    
f6 = rule_function(vehicle_time_to_collision, max)

# TODO: haussdorf distance does not work, use distance + intersection perhaps?

def stay_in_drivable_area(realization, step, **kwargs):
    ego = realization.ego
    ego_state = ego.get_state(step)
    drivable_region = realization.network.drivableRegion.polygons
    
    difference = ego_state.polygon.difference(drivable_region)
    area = difference.area
    
    distance = shapely.distance(drivable_region, ego_state.polygon)
    violation = area + distance**2
    
    return violation

f3 = rule_function(stay_in_drivable_area, max)

'''
def rule_stay_in_drivable_area(realization, start_index=None, end_index=None):
    violation_history = []
    if start_index is None:
        start_index = 0

    max_steps = realization.max_steps
    
    if end_index is None:
        end_index = max_steps
        
        
    network = realization.network
    drivable_region = network.drivableRegion
    ego = realization.ego
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
'''

def vru_clearance(realization, step, **kwargs):
    clearance_type = kwargs.get("clearance_type", "VRU")
    world_state = realization.get_world_state(step)
    threshold = kwargs.get("threshold", 2)
    ego_state = world_state.ego_state
    vru_states = world_state.vru_states
    drivable_area = realization.network.drivableRegion.polygons
    if clearance_type == "VRU_on_road":
        vru_states = [vru for vru in vru_states if shapely.intersects(vru.polygon, drivable_area)]
    elif clearance_type == "VRU_off_road":
        vru_states = [vru for vru in vru_states if not shapely.intersects(vru.polygon, drivable_area)]
    else:
        return ValueError(f"Invalid clearance type: {clearance_type}. Must be 'VRU_on_road', 'VRU_off_road' or 'vehicle'.")
    
    violation = 0
    for vru_state in vru_states:
        vru_polygon = vru_state.polygon
        distance = shapely.distance(ego_state.polygon, vru_polygon)
        violation = max(violation, threshold - distance)
    return violation
    
def vru_on_road_clearance(realization, step, **kwargs):
    return vru_clearance(realization, step, clearance_type="VRU_on_road", **kwargs)

def vru_off_road_clearance(realization, step, **kwargs):
    return vru_clearance(realization, step, clearance_type="VRU_off_road", **kwargs)



f8 = rule_function(vru_off_road_clearance, max)

f9 = rule_function(vru_on_road_clearance, max)
'''
def vru_clearance(realization, on_road=False, threshold = 2, start_index=None, end_index=None):
    violation_history = []
    
    if start_index is None:
        start_index = 0
        
    if end_index is None:
        end_index = realization.max_steps
    ego = realization.ego
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
'''


def vru_acknowledgement(realization, step, **kwargs):
    proximity = kwargs.get("proximity", 5)
    threshold = kwargs.get("threshold", 0)
    timesteps = kwargs.get("timesteps", 30)
    ego = realization.ego
    vrus = realization.VRUs
    
    if step == 0:
        return 0
    
    violation = 0
    
    
    
    for vru in vrus:
        for i in range(step, min(step + timesteps, len(ego.trajectory))):
            ego_state = ego.get_state(i)
            vru_state = vru.get_state(i)
            
            dist = polygon_distance(ego_state, vru_state)
            if dist < proximity:
                ego_before_state = ego.get_state(step-1)
                ego_before_velocity = ego_before_state.velocity
                ego_after_state = ego.get_state(step)
                ego_after_velocity = ego_after_state.velocity
                vru_pos = vru.get_state(step).position
                
                relative_position = vru_pos - ego_after_state.position
                relative_position = relative_position.normalized()
                acceleration = (ego_after_velocity - ego_before_velocity) / realization.delta
                
                acceleration_projection = float(acceleration.dot(relative_position)) * relative_position
                violation = max(violation, acceleration_projection.norm() - threshold)
                break
    
    return violation
        
        
        
    
f5 = rule_function(vru_acknowledgement, max)
    

'''
def vru_acknowledgement(realization, proximity=5, threshold=5,  timesteps=10, start_index=None, end_index=None):
    if start_index is None:
        start_index = 0
    
    if end_index is None:
        end_index = realization.max_steps
    
    
    violation_history = []
    ego = realization.ego
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
'''
# TODO: vehicle yielding rule based on adv vehicle decelerations

def correct_side(realization, step, **kwargs):
    ego = realization.ego
    ego_state = ego.get_state(step)
    lane = ego_state.lane
    if lane is None:
        return 0
    ego_orientation = ego_state.orientation.yaw
    lane_orientation = lane.orientation.value(ego_state.position)

    if math.cos(lane_orientation - ego_orientation) < 0:
        return 1
    return 0

f7 = rule_function(correct_side, sum)
'''
def road_correct_side(realization, start_index=None, end_index=None):
    violation_history = []
    if start_index is None:
        start_index = 0

    max_steps = realization.max_steps
    
    if end_index is None:
        end_index = max_steps
        
    network = realization.network
    ego = realization.ego
    violation_score = 0
    
    for i in range(start_index, end_index):
        violation = 0
        state = ego.get_state(i)
        ego_position = state.position
        ego_orientation = state.orientation
        try:
            road_orientation_field = network.roadAt(ego_position).orientation
        except:# road not found
            violation_history.append(0)
            continue
        road_orientation = road_orientation_field.value(ego_position)
        road_yaw = road_orientation.yaw
        ego_yaw = ego_orientation.yaw
        violation = 1 if math.cos(road_yaw - ego_yaw) < 0 else 0
        violation_history.append(violation)
    violation_score = sum(violation_history)
    return violation_score, violation_history
'''
def speed_limit(realization, step, threshold=15): # speed limit
    ego_state = realization.get_ego().get_state(step)
    ego_velocity = ego_state.velocity.norm()
    if ego_state.lane is None or ego_state.lane.speedLimit is None:
        speed_limit = threshold
    else:
        speed_limit = ego_state.lane.speedLimit
    
    return max(0, ego_velocity - speed_limit)**2

f15 = rule_function(speed_limit, max)

def lane_keeping(realization, step):
    if step == 0:
        return 0
    ego = realization.ego
    ego_state = ego.get_state(step)
    ego_prev_state = ego.get_state(step - 1)
    ego_lane = ego_state.lane
    ego_prev_lane = ego_prev_state.lane
    
    if ego_lane == ego_prev_lane:
        return 0
    elif (ego_prev_lane is None and ego_lane is not None) or (ego_prev_lane is not None and ego_lane is None): # TODO: ask about this
        return 1 
    else:
        for maneuver in ego_prev_lane.maneuvers:
            if maneuver.endLane == ego_lane or maneuver.connectingLane == ego_lane:
                return 0
        return 1

f17 = rule_function(lane_keeping, sum)    

def jerk(realization, step):
    if step == 0 or step == 1:
        return 0
    ego = realization.ego
    ego_state = ego.get_state(step)
    ego_prev_state = ego.get_state(step - 1)
    ego_prev_prev_state = ego.get_state(step - 2)
    ego_velocity = ego_state.velocity
    ego_prev_velocity = ego_prev_state.velocity
    ego_prev_prev_velocity = ego_prev_prev_state.velocity
    acceleration = (ego_velocity - ego_prev_velocity) / realization.delta
    prev_acceleration = (ego_prev_velocity - ego_prev_prev_velocity) / realization.delta
    jerk_value = (acceleration - prev_acceleration).norm()
    return jerk_value

f20 = rule_function(jerk, max)


def longitudinal_acceleration(realization, step):
    if step == 0:
        return 0
    ego = realization.ego
    ego_state = ego.get_state(step)
    ego_orientation = ego_state.orientation.yaw
    ego_orientation_vector = Vector(math.cos(ego_orientation), math.sin(ego_orientation), 0).normalized()
    ego_velocity = ego_state.velocity
    ego_prev_state = ego.get_state(step - 1)
    ego_prev_velocity = ego_prev_state.velocity
    ego_acceleration = (ego_velocity - ego_prev_velocity) / realization.delta
    longitudinal_acceleration = ego_acceleration.dot(ego_orientation_vector)
    return longitudinal_acceleration.norm()


f21 = rule_function(longitudinal_acceleration, max)

def lateral_acceleration(realization, step):
    if step == 0:
        return 0
    ego = realization.ego
    ego_state = ego.get_state(step)
    ego_orientation = ego_state.orientation.yaw + math.pi/2 # perpendicular to the orientation
    ego_orientation_vector = Vector(math.cos(ego_orientation), math.sin(ego_orientation), 0).normalized()
    ego_velocity = ego_state.velocity
    ego_prev_state = ego.get_state(step - 1)
    ego_prev_velocity = ego_prev_state.velocity
    ego_acceleration = (ego_velocity - ego_prev_velocity) / realization.delta
    lateral_acceleration = ego_acceleration.dot(ego_orientation_vector)
    return abs(lateral_acceleration.norm())

f22 = rule_function(lateral_acceleration, max)

def lane_centering(realization, step): # lane centering
    ego = realization.ego
    ego_state = ego.get_state(step)
    ego_pos = ego_state.position
    ego_lane = ego_state.lane
    if ego_lane is None:
        return 0
    centerline = ego_lane.centerline.lineString
    # double check shapely distance function for sparse centerline
    ego_pos_point = shapely.Point(ego_pos.x, ego_pos.y)
    distance = centerline.distance(ego_pos_point)
    return distance


f18 = rule_function(lane_centering, sum)
# TODO: vehicle yielding rule10, parked vehicle rule 14, turn signal rule 16
# TODO: lane keeping rule 17, following distance rule 19

def trajectory_to_lineString(trajectory):
    return shapely.LineString([(state.position.x, state.position.y) for state in trajectory])
     
def project_point_to_linestring(ls: shapely.LineString, point: shapely.Point):
    x = ls.project(point)
    projected_point = ls.interpolate(x)
    y = projected_point.distance(point)
    
    return x, y, projected_point

def project_polygon_to_linestring(ls: shapely.LineString, polygon: shapely.Polygon):
    #print(ls)
    #print(polygon.centroid)
    x = ls.project(polygon.centroid)
    projected_point = ls.interpolate(x)
    y = ls.distance(polygon)
    return x, y, projected_point   

def polygon_distance(x_state, y_state):
    x_polygon = x_state.polygon
    y_polygon = y_state.polygon
    return shapely.distance(x_polygon, y_polygon)

def next_lane(state): # find the next lane in the object's trajectory
    obj = state.object
    current_lane = state.lane
    
    i = state.step + 1
    while i < len(obj.trajectory):
        next_state = obj.get_state(i)
        if next_state.lane != current_lane: # lane changed
            return next_state.lane
        i += 1
    
def proximity_filter(object_states, ego_state, threshold):
    """
    Filters the object states based on proximity to the ego vehicle.
    :param object_states: List of object states to filter.
    :param ego_state: The state of the ego vehicle.
    :param threshold: The distance threshold for proximity.
    :return: List of candidate objects within the proximity threshold.
    """
    candidate_objects = []
    for obj_state in object_states:
        distance = (obj_state.position - ego_state.position).norm()
        if distance < threshold:
            candidate_objects.append(obj_state)
    return candidate_objects
    
def to_shapely_point(vector):
    """
    Converts a Vector to a shapely Point.
    :param vector: The Vector to convert.
    :return: A shapely Point representing the vector's position.
    """
    return shapely.Point(vector.x, vector.y)

def get_front_vehicle(ego_state, candidate_object_states, ls, margin):
    """
    Finds the front vehicle in the candidate objects based on the ego vehicle's trajectory.
    :param ego_state: The state of the ego vehicle.
    :param candidate_objects: List of candidate objects to check.
    :param ls: The LineString representing the ego vehicle's trajectory.
    :param margin: Margin to consider for the front vehicle.
    :return: The front vehicle object or None if not found.
    """
    front_state = None
    min_distance = float("inf")
    
    for obj_state in candidate_object_states:
        obj_polygon = obj_state.polygon
        x, y, projected_point = project_polygon_to_linestring(ls, obj_polygon)
        
        if y < (ego_state.object.dimensions[0]/2 + margin) and x < min_distance:
            min_distance = x
            front_state = obj_state
    
    return front_state
        
def trajectory_clearance_rule(realization, step, **kwargs):
    side = kwargs.get("side", "front")
    margin = kwargs.get("margin", 0.5)
    threshold = kwargs.get("threshold", 2)
    if step == len(realization) - 1:
        return 0
    world_state = realization.get_world_state(step)
    ego_state = world_state.ego_state
    other_vehicle_states = world_state.other_vehicle_states
    
    candidate_object_states = proximity_filter(other_vehicle_states, ego_state, kwargs.get("proximity", 10))
    if len(candidate_object_states) == 0:
        return 0
    
    ls = trajectory_to_lineString(ego_state.object.trajectory[step:])
    if side == "front":
        return trajectory_front_clearance(ego_state, candidate_object_states, ls, threshold, margin)
    elif side in ["left", "right"]:
        return trajectory_side_clearance(ego_state, candidate_object_states, ls, side, threshold, margin)
    else:
        raise ValueError(f"Invalid side: {side}. Must be 'front', 'left' or 'right'.")

def trajectory_front_clearance(ego_state, candidate_object_states, ls, threshold, margin):
    front_vehicle_state = get_front_vehicle(ego_state, candidate_object_states, ls, margin)
    if front_vehicle_state is None:
        return 0
    violation = max(0, threshold - polygon_distance(ego_state, front_vehicle_state))
    return violation

def front_clearance_trajectory(realization, step, **kwargs):
    return trajectory_clearance_rule(realization, step, side="front", **kwargs)

f11_b = rule_function(front_clearance_trajectory, max)

def get_side_vehicles(ego_state, candidate_object_states, ls, side, margin):
    orientation = ego_state.orientation.yaw
    orientation_vector = Vector(math.cos(orientation), math.sin(orientation), 0).normalized()
    side_vehicles = []
    for obj_state in candidate_object_states: # decide which side the vehicle is on
        obj_pos = obj_state.position
        obj_vector = (obj_pos - ego_state.position).normalized()
        angle = orientation_vector.angleWith(obj_vector)
        x, y, projected_point = project_polygon_to_linestring(ls, obj_state.polygon)
        
        if y < (ego_state.object.dimensions[0]/2 + margin):
            continue

        if math.pi >= angle >= 0 and side == "left":
            side_vehicles.append(obj_state)
        elif -math.pi < angle < 0 and side == "right":
            side_vehicles.append(obj_state)
    return side_vehicles

def trajectory_side_clearance(ego_state, candidate_object_states, ls, side, threshold, margin):

    side_vehicles = get_side_vehicles(ego_state, candidate_object_states, ls, side, margin)

    if len(side_vehicles) == 0:
        return 0
    
    violation = 0
    for obj_state in side_vehicles:
        violation = max(0, threshold - polygon_distance(ego_state, obj_state), violation)
        
    return violation

def left_clearance_trajectory(realization, step, **kwargs):
    return trajectory_clearance_rule(realization, step, side="left", **kwargs)
    
def right_clearance_trajectory(realization, step, **kwargs):
    return trajectory_clearance_rule(realization, step, side="right", **kwargs)

f12_b = rule_function(left_clearance_trajectory, max)

f13_b = rule_function(right_clearance_trajectory, max)

def buffer_get_front_vehicle(candidate_object_states, front_ls):

    front_state = None
    min_distance = float("inf")
    
    for obj_state in candidate_object_states:
        obj_polygon = obj_state.polygon
        x, y, projected_point = project_polygon_to_linestring(front_ls, obj_polygon)
        if x < min_distance:
            min_distance = x
            front_state = obj_state
    
    return front_state

def buffer_front_clearance(ego_state, candidate_object_states, front_ls, threshold):
    front_state = buffer_get_front_vehicle(candidate_object_states, front_ls)
    
    if front_state is None:
        return 0
    
    violation = max(0, threshold - polygon_distance(ego_state, front_state))
    return violation

def buffer_side_clearance(ego_state, candidate_object_states, side, margin, threshold, proximity):
    buffer_size = ego_state.object.dimensions[0]/2 + margin
    side_buffer_size = proximity
    if side == "right":
        side_buffer_size *= -1
        
    ego_ls = trajectory_to_lineString(ego_state.object.trajectory)
    traj_buffer_polygon = ego_ls.buffer(buffer_size, cap_style='flat')
    side_buffer_polygon = ego_ls.buffer(side_buffer_size, cap_style='flat', single_sided=True)
    side_buffer_polygon = side_buffer_polygon.difference(traj_buffer_polygon)

    violation = 0
    for obj_state in candidate_object_states:
        if side_buffer_polygon.intersects(obj_state.polygon):
            violation = max(0, threshold - polygon_distance(ego_state, obj_state), violation)            

    return violation

def buffer_clearance_rule(realization, step, **kwargs):
    world_state = realization.get_world_state(step)
    ego_state = world_state.ego_state
    ego_width = ego_state.object.dimensions[0]
    if step == len(realization) - 1 or step == 0:
        return 0
    threshold = kwargs.get("threshold", 2)
    other_vehicle_states = world_state.other_vehicle_states
    proximity = kwargs.get("proximity", 10)
    side = kwargs.get("side", "front")
    margin = kwargs.get("margin", 0.5)
    candidate_object_states = proximity_filter(other_vehicle_states, ego_state, proximity)
    if len(candidate_object_states) == 0:
        return 0
    
    front_trajectory = ego_state.object.trajectory[step:]
    behind_trajectory = ego_state.object.trajectory[:step+1]
    #print(front_trajectory)
    front_ls = trajectory_to_lineString(front_trajectory)
    front_buffer_polygon = front_ls.buffer(ego_width/2 + margin, cap_style='flat')
    behind_ls = trajectory_to_lineString(behind_trajectory)
    behind_buffer_polygon = behind_ls.buffer(ego_width/2 + margin, cap_style='flat')
    
    front_vehicles = []
    side_vehicles = []
    
    for obj_state in candidate_object_states: # decide which side the vehicle is on
        if front_buffer_polygon.intersects(obj_state.polygon):
            front_vehicles.append(obj_state)
        elif behind_buffer_polygon.intersects(obj_state.polygon):
            continue
        else:
            side_vehicles.append(obj_state)
    
    if side == "front":
        return buffer_front_clearance(ego_state, front_vehicles, front_ls, threshold)
    
    elif side in ["left", "right"]:
        return buffer_side_clearance(ego_state, side_vehicles, side, margin, threshold, proximity)
    
    else:
        raise ValueError(f"Invalid side: {side}. Must be 'front', 'left' or 'right'.")
            
def front_clearance_buffer(realization, step, **kwargs):
    return buffer_clearance_rule(realization, step, side="front", **kwargs)

f11_a = rule_function(front_clearance_buffer, max)

def left_clearance_buffer(realization, step, **kwargs):
    return buffer_clearance_rule(realization, step, side="left", **kwargs)

f12_a = rule_function(left_clearance_buffer, max)

def right_clearance_buffer(realization, step, **kwargs):
    return buffer_clearance_rule(realization, step, side="right", **kwargs)

f13_a = rule_function(right_clearance_buffer, max)