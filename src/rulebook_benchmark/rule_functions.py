from trimesh.transformations import compose_matrix
from scenic.core.regions import MeshVolumeRegion, EmptyRegion
import shapely
from rulebook_benchmark.realization import Realization, State, RealizationObject
import math
from scenic.core.vectors import Vector
from rulebook_benchmark.rulebook import Rule


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
        max_steps = realization.max_steps
        
        if end_index is None:
            end_index = max_steps
            
        violation_score = 0
        for i in range(start_index, end_index + 1):
            violation_score, carryover = calculate_violation(realization, i, carryover=carryover, **parameters)
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


        


def rule_15(realization, step, threshold=10): # speed limit
    ego_velocity = realization.get_ego_state(step).velocity.norm()
    return max(0, ego_velocity - threshold)**2

def rule_18(realization, step): # lane centering
    ego_state = realization.get_ego_state(step)
    ego_pos = ego_state.position
    
    centerline = realization.network.laneAt(ego_pos).centerline.lineString
    # double check shapely distance function for sparse centerline
    ego_pos_point = shapely.Point(ego_pos.x, ego_pos.y)
    distance = centerline.distance(ego_pos_point)
    return distance





# TODO: vehicle yielding rule10, VRU TTC rule 4, vehicle TTC rule6, parked vehicle rule 14, turn signal rule 16
# TODO: lane keeping rule 17, following distance rule 19

def trajectory_to_lineString(trajectory):
    return shapely.LineString([(state.position.x, state.position.y) for state in trajectory])

        
def project_point_to_linestring(ls: shapely.LineString, point: shapely.Point):
    x = ls.project(point)
    projected_point = ls.interpolate(x)
    y = projected_point.distance(point)
    
    return x, y, projected_point

def project_polygon_to_linestring(ls: shapely.LineString, polygon: shapely.Polygon):
    x = ls.project(polygon.centroid)
    projected_point = ls.interpolate(x)
    y = ls.distance(polygon)
    return x, y, projected_point
    

def polygon_distance(x_state, y_state):
    x_polygon = x_state.object.get_polygon(x_state)
    y_polygon = y_state.object.get_polygon(y_state)
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
        
        if y < (ego_state.obj.dimensions[0]/2 + margin) and x < min_distance:
            min_distance = x
            front_state = obj_state
    
    return front_state
        

def trajectory_clearance_rule(realization, step, **kwargs):
    world_state = realization.get_world_state(step)
    ego_state = world_state.ego_state
    other_vehicle_states = world_state.other_vehicle_states
    
    candidate_object_states = proximity_filter(other_vehicle_states, ego_state, kwargs.get("proximity", 10))
    if len(candidate_object_states) == 0:
        return 0
    
    ls = trajectory_to_lineString(ego_state.object.trajectory, step)
    if kwargs.get("side", "front") == "front":
        return trajectory_front_clearance(ego_state, candidate_object_states, ls, **kwargs)
    else:
        return trajectory_side_clearance(ego_state, candidate_object_states, ls, **kwargs)


def trajectory_front_clearance(ego_state, candidate_object_states, ls, **kwargs):
    front_vehicle_state = get_front_vehicle(ego_state, candidate_object_states, ls, kwargs.get("margin", 1))
    if front_vehicle_state is None:
        return 0
    violation = max(0, kwargs.get("threshold", 5) - polygon_distance(ego_state, front_vehicle_state))
    return violation


def get_side_vehicles(ego_state, candidate_object_states, ls, side, margin):
    orientation = ego_state.orientation.yaw
    orientation_vector = Vector(math.cos(orientation), math.sin(orientation), 0).normalized()
    side_vehicles = []
    for obj_state in candidate_object_states: # decide which side the vehicle is on
        obj_pos = obj_state.position
        obj_vector = (obj_pos - ego_state.position).normalized()
        angle = orientation_vector.angleWith(obj_vector)
        x, y, projected_point = project_polygon_to_linestring(ls, obj_state.polygon)
        
        if y < (ego_state.obj.dimensions[0]/2 + margin):
            continue

        if math.pi > angle > 0 and side == "left":
            side_vehicles.append(obj_state)
        elif -math.pi < angle < 0 and side == "right":
            side_vehicles.append(obj_state)
        elif angle == 0 or angle == math.pi:
            # vehicle is directly in front or behind, ignore
            continue
    return side_vehicles

def trajectory_side_clearance(ego_state, candidate_object_states, ls, **kwargs):
    side = kwargs.get("side", "left")
    margin = kwargs.get("margin", 1)
    threshold = kwargs.get("threshold", 5)
        
    side_vehicles = get_side_vehicles(ego_state, candidate_object_states, ls, side, margin)
    
    if len(side_vehicles) == 0:
        return 0
    
    violation = 0
    for obj_state in side_vehicles:
        violation = max(0, threshold - polygon_distance(ego_state, obj_state), violation)
        
    return violation
    
    
    


    
    

def buffer_clearance_rule(realization, step, **kwargs):
    world_state = realization.get_world_state(step)
    ego_state = world_state.ego_state
    other_vehicle_states = world_state.other_vehicle_states
    side = kwargs.get("side", "front")    
    candidate_object_states = proximity_filter(other_vehicle_states, ego_state, kwargs.get("proximity", 10))
    if len(candidate_object_states) == 0:
        return 0
    
    front_trajectory = ego_state.object.trajectory[step:]
    front_ls = trajectory_to_lineString(front_trajectory)
    front_buffer_polygon = front_ls.buffer(kwargs.get("buffer", 1), cap_style=shapely.CAP_STYLE.flat)
    


        
    



      


     
    
    
    
    
        
        

    
    


