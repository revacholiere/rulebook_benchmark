from functools import cached_property
from trimesh.transformations import compose_matrix
from scenic.core.regions import MeshVolumeRegion, EmptyRegion
import shapely
from rulebook_benchmark.realization import Realization, State, RealizationObject
import math
from scenic.core.vectors import Vector
from rulebook_benchmark.rulebook import Rule
import numpy as np
from rulebook_benchmark.utils import normalize_vector, intersects, angle_between, continuous_ttc
from numpy.linalg import norm




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



class Rule:
    def __init__(self, calculate_violation, aggregation_method, **kwargs):
        self.calculate_violation = calculate_violation
        self.aggregation_method = aggregation_method
        self.parameters = kwargs

    def __call__(self, handler, step, **runtime_params):
        # merge init parameters and runtime ones
        params = {**self.parameters, **runtime_params}
        return self.calculate_violation(handler, step, **params)



class RuleEngine:
    def __init__(self, rules):
        # rules is a dict: {"rule_name": Rule(...), ...}
        self.rules = rules

    def evaluate(self, handler, start_index=None, end_index=None, **runtime_params):
        realization = handler.realization
        max_steps = len(realization) - 1

        if start_index is None:
            start_index = 0
        if end_index is None:
            end_index = max_steps

        # initialize results per rule
        results = {name: Result(aggregation_method=rule.aggregation_method)
                   for name, rule in self.rules.items()}

        # pad initial history
        for res in results.values():
            res.violation_history += [0] * start_index

        # step loop
        for step in range(start_index, end_index + 1):
            for name, rule in self.rules.items():
                violation_score = rule(handler, step, **runtime_params)
                results[name].add(violation_score)

        # pad final history
        for res in results.values():
            res.violation_history += [res.total_violation] * (max_steps - end_index)

        return results


def kinetic_energy_loss(ego_velocity_before, ego_velocity_after, adv_velocity_before, adv_velocity_after, ego_mass, adv_mass):
    ego_loss = 0.5 * ego_mass * (np.linalg.norm(ego_velocity_before) ** 2 - np.linalg.norm(ego_velocity_after) ** 2)
    adv_loss = 0.5 * adv_mass * (np.linalg.norm(adv_velocity_before) ** 2 - np.linalg.norm(adv_velocity_after) ** 2)

    return ego_loss + adv_loss

def momentum_loss(ego_velocity_before, ego_velocity_after, adv_velocity_before, adv_velocity_after, ego_mass, adv_mass):
    ego_momentum_loss = np.linalg.norm(ego_mass * (ego_velocity_after - ego_velocity_before))
    adv_momentum_loss = np.linalg.norm(adv_mass * (adv_velocity_after - adv_velocity_before))

    return ego_momentum_loss + adv_momentum_loss

def generalized_collision(handler, collision_timeline, states, step, ego_mass, adv_mass, momentum):
    violation = 0
    for state in states:
        uid = state.uid
        if uid not in collision_timeline:
            continue
        collisions = collision_timeline[uid]
        for collision in collisions:
            before_collision, after_collision = collision

            if before_collision > step:
                break
            elif before_collision < step:
                continue
            else:
                prev_state = handler(before_collision).ego_state
                after_state = handler(after_collision).ego_state

                adv_prev_state = handler(before_collision).world_state[uid]
                adv_after_state = handler(after_collision).world_state[uid]

                if momentum:
                    curr_violation = max(0, momentum_loss(
                        ego_velocity_before=prev_state.velocity,
                        ego_velocity_after=after_state.velocity,
                        adv_velocity_before=adv_prev_state.velocity,
                        adv_velocity_after=adv_after_state.velocity,
                        ego_mass=ego_mass,
                        adv_mass=adv_mass
                    ))

                else:
                    curr_violation = max(0, kinetic_energy_loss(
                        ego_velocity_before=prev_state.velocity,
                        ego_velocity_after=after_state.velocity,
                        adv_velocity_before=adv_prev_state.velocity,
                        adv_velocity_after=adv_after_state.velocity,
                        ego_mass=ego_mass,
                        adv_mass=adv_mass
                    ))

                violation += curr_violation

    return violation


def vru_collision(handler, step, car_mass=1500, vru_mass=70, momentum = False):
    vru_states = handler(step).vrus_in_proximity
    return generalized_collision(handler, handler.collision_timeline, vru_states, step, car_mass, vru_mass, momentum)


def vehicle_collision(handler, step, car_mass=1500, momentum = False):
    vehicle_states = handler(step).vehicles_in_proximity
    return generalized_collision(handler, handler.collision_timeline, vehicle_states, step, car_mass, car_mass, momentum)




def cross2d(a, b):
    return a[0] * b[1] - a[1] * b[0]

def lines_intersect(p1, p2, q1, q2):
    r = p2 - p1
    s = q2 - q1
    denom = cross2d(r, s)
    if abs(denom) < 1e-9:  # parallel
        return False, (None, None)
    t = cross2d(q1 - p1, s) / denom
    u = cross2d(q1 - p1, r) / denom
    return True, (t, u)

def early_ttc(ego_pos, ego_vel, adv_pos, adv_vel, threshold, times=3):
    horizon = threshold * times
    ego_end = ego_pos + ego_vel * horizon
    adv_end = adv_pos + adv_vel * horizon

    intersect, (t, u) = lines_intersect(ego_pos, ego_end, adv_pos, adv_end)
    if not intersect:
        return False  # no intersection ever

    # if intersection happens after horizon, skip expensive TTC
    if t > horizon or u > horizon or t < 0 or u < 0:
        return False

    return True  # possible interaction, run continuous_ttc

    
    



def vru_ttc(handler, step, threshold=1.0):
    pool = handler(step)
    ego_state = pool.ego_state
    ego_velocity = ego_state.velocity
    ego_position = ego_state.position
    
    violation = 0
    
    for state in pool.vru_states:
        obj_velocity = state.velocity
        obj_pos = state.position
        if not early_ttc(ego_position, ego_velocity, obj_pos, obj_velocity, threshold):
            continue
        
        v_rel = (obj_velocity[0] - ego_velocity[0], obj_velocity[1] - ego_velocity[1])
        ttc = continuous_ttc(ego_state.coords_np, state.coords_np, v_rel, threshold)
        if ttc is not None:
            violation = max(violation, threshold - ttc)

    return violation


def vehicle_ttc(handler, step, threshold=0.8):
    pool = handler(step)
    ego_state = pool.ego_state
    ego_velocity = ego_state.velocity
    ego_position = ego_state.position
    ego_polygon = ego_state.polygon
    
    violation = 0
    
    for state in pool.other_vehicle_states:

        obj_velocity = state.velocity
        obj_polygon = state.polygon
        obj_pos = state.position
        
        if not early_ttc(ego_position, ego_velocity, obj_pos, obj_velocity, threshold):
            continue
        v_rel = (obj_velocity[0] - ego_velocity[0], obj_velocity[1] - ego_velocity[1])
        ttc = continuous_ttc(ego_polygon.exterior.coords[:-1], obj_polygon.exterior.coords[:-1], v_rel, threshold)
        if ttc is not None:
            violation = max(violation, threshold - ttc)

    return violation



f1 = Rule(vru_collision, max)
f2 = Rule(vehicle_collision, max)

f4 = Rule(vru_ttc, max, threshold=1.0)
f6 = Rule(vehicle_ttc, max, threshold=0.8)

def stay_in_drivable_area(handler, step, **kwargs):
    ego = handler.ego
    ego_state = ego.get_state(step)
    drivable_region = handler.network.drivableRegion.polygons

    difference = ego_state.polygon.difference(drivable_region)
    area = difference.area
    
    distance = shapely.distance(drivable_region, ego_state.polygon)
    violation = area + distance**2
    
    return violation

f3 = Rule(stay_in_drivable_area, max)


def vru_clearance(handler, step, on_road=False, threshold = 2):
    pool = handler(step)
    vru_states = pool.vrus_in_proximity
    violation = 0
    for vru_state in vru_states:
        if on_road:
            if vru_state.lane is not None:
                distance = pool.distance(vru_state)
            else:
                continue
        else:
            distance = pool.distance(vru_state)

        violation = max(violation, threshold - distance)

    return violation


f8 = Rule(vru_clearance, max, on_road=False, threshold=2)
f9 = Rule(vru_clearance, max, on_road=True, threshold=2)

    
def vru_acknowledgement(handler, step, threshold = 0, proximity = 1, timesteps = 20):
    candidates = []
    violation = 0
    for i in range(step, min(step + timesteps, len(handler.realization))):
        pool = handler(i)
        for state in pool.vru_states:
            distance = pool.distance(state)
            if distance < proximity:
                candidates.append(state.uid)


    pool = handler(step)
    ego_acceleration = pool.ego_state.acceleration
    for uid in candidates:
        state = pool.world_state[uid]
        # project ego acceleration onto the direction from ego to vru
        relative_position = state.position - pool.ego_state.position
        relative_position = normalize_vector(relative_position)
        ego_acceleration_projected = np.dot(ego_acceleration, relative_position)
        violation = max(0, ego_acceleration_projected - threshold, violation)
        
    return violation

f5 = Rule(vru_acknowledgement, max, threshold = -1, proximity = 1, timesteps = 20)
# TODO: vehicle yielding rule based on adv vehicle decelerations

def correct_side(handler, step, **kwargs):
    ego_state = handler(step).ego_state
    lane = ego_state.lane
    if lane is None:
        return 0
    ego_orientation = ego_state.orientation.yaw
    lane_orientation = lane.orientation.value(ego_state.position)

    if math.cos(lane_orientation - ego_orientation) < 0:
        return 1
    return 0

f7 = Rule(correct_side, sum)

def speed_limit(handler, step, threshold=15): # speed limit
    ego_state = handler(step).ego_state
    ego_velocity = norm(ego_state.velocity)
    if ego_state.lane is None or ego_state.lane.speedLimit is None:
        speed_limit = threshold
    else:
        speed_limit = ego_state.lane.speedLimit
    
    return max(0, ego_velocity - speed_limit)**2

f15 = Rule(speed_limit, max)

def lane_keeping(handler, step):
    if step == 0:
        return 0
    ego_state = handler(step).ego_state
    ego_prev_state = handler(step - 1).ego_state
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

f17 = Rule(lane_keeping, sum)

def jerk(handler, step):
    if step == 0:
        return 0

    ego_prev_state = handler(step-1).ego_state
    ego_state = handler(step).ego_state
    
    jerk_value = norm(ego_state.acceleration - ego_prev_state.acceleration)
    return jerk_value

f20 = Rule(jerk, sum)


def longitudinal_acceleration(handler, step):
    if step == 0:
        return 0
    ego_state = handler(step).ego_state
    ego_orientation = ego_state.orientation.yaw
    ego_orientation_vector = normalize_vector(np.array([math.cos(ego_orientation), math.sin(ego_orientation)]))
    ego_acceleration = ego_state.acceleration
    longitudinal_acceleration = ego_acceleration.dot(ego_orientation_vector)
    return norm(longitudinal_acceleration)


f21 = Rule(longitudinal_acceleration, max)

def lateral_acceleration(handler, step):
    if step == 0:
        return 0
    ego_state = handler(step).ego_state
    ego = ego_state.object
    ego_velocity = ego_state.velocity
    turning_radius = ego.length / math.sin(ego.steer * math.pi / 2)
    lateral_acceleration = norm(ego_velocity) ** 2 / turning_radius if turning_radius != 0 else 0
    return abs(lateral_acceleration)

f22 = Rule(lateral_acceleration, max)

def lane_centering(handler, step, buffer=0.3): # lane centering
    ego_state = handler(step).ego_state
    ego_pos = ego_state.position
    ego_lane = ego_state.lane
    if ego_lane is None:
        return 0
    centerline = ego_lane.centerline.lineString
    if buffer > 0:
        centerline = centerline.buffer(buffer)
    # double check shapely distance function for sparse centerline
    ego_pos_point = shapely.Point(ego_pos)
    distance = centerline.distance(ego_pos_point)
    return distance


f18 = Rule(lane_centering, sum, buffer=0.3)
# TODO: vehicle yielding rule10, parked vehicle rule 14, turn signal rule 16
# TODO: lane keeping rule 17, following distance rule 19



def front_clearance(handler, step, threshold = 0.8):
    if step == len(handler.realization) - 1:
        return 0
    pool = handler(step)
    front_ls = pool.trajectory_front_linestring
    ego_width = handler.ego.width

    states = pool.vehicles_in_proximity
    
    violation = 0
    for state in states:
        if front_ls.distance(state.polygon) < ego_width/2:
            distance = pool.distance(state)
            violation = max(violation, threshold - distance)

    return violation



def side_clearance(handler, step, left = True, threshold = 0.8):
    pool = handler(step)
    if step == 0:
        ls = pool.trajectory_front_linestring
    elif step == len(handler.realization) - 1:
        ls = pool.trajectory_behind_linestring
    else:
        ls = shapely.union(pool.trajectory_front_linestring, pool.trajectory_behind_linestring)


    violation = 0
    states = pool.vehicles_in_proximity
    ego_state = pool.ego_state
    width = handler.ego.width
    ego_heading_vector = normalize_vector(np.array([math.cos(ego_state.orientation.yaw), math.sin(ego_state.orientation.yaw)]))
    for state in states:
        if ls.distance(state.polygon) > width/2:
            ego_to_object = normalize_vector(state.position - ego_state.position)
            angle = angle_between(ego_heading_vector, ego_to_object)
            if (angle >= 0 and left) or (angle < 0 and not left):
                violation = max(violation, threshold - pool.distance(state))
                
    return violation


f11 = Rule(front_clearance, max, threshold=0.8)

f12 = Rule(side_clearance, max, left=True, threshold=0.8)

f13 = Rule(side_clearance, max, left=False, threshold=0.8)






def clearance_vector_based(handler, step, threshold=0.8, front_angle=math.radians(30), side="front"):
    pool = handler(step)
    ego_state = pool.ego_state
    states = pool.vehicles_in_proximity
    
    violation = 0
    
    ego_heading_vector = normalize_vector(np.array([math.cos(ego_state.orientation.yaw), math.sin(ego_state.orientation.yaw)]))
    
    for state in states:
        state_vector = normalize_vector(np.array([math.cos(state.orientation.yaw), math.sin(state.orientation.yaw)]))
        angle = angle_between(ego_heading_vector, state_vector)
        if (abs(angle) <= front_angle / 2 and side == "front") or (angle < 0 and side == "right") or (angle >= 0 and side == "left"):
            violation = max(violation, threshold - pool.distance(state))

    return violation

f11_v = Rule(clearance_vector_based, max, threshold=0.8)
f12_v = Rule(clearance_vector_based, max, side="left", threshold=0.8)
f13_v = Rule(clearance_vector_based, max, side="right", threshold=0.8)

'''

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
    threshold = kwargs.get("threshold", 0.5)
    other_vehicle_states = world_state.other_vehicle_states
    proximity = kwargs.get("proximity", 5)
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
    #ego_last_state = realization.get_ego().get_state((len(realization) - 1))
    #front_buffer_polygon = front_buffer_polygon.union(ego_last_state.polygon)
    behind_ls = trajectory_to_lineString(behind_trajectory)
    ego_first_state = realization.get_ego().get_state(0)
    behind_buffer_polygon = behind_ls.buffer(ego_width/2 + margin, cap_style='flat')
    #behind_buffer_polygon = behind_buffer_polygon.union(ego_first_state.polygon)

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

'''


'''


def clearance_vector_based(realization, step, **kwargs):
    front_angle = kwargs.get("front_angle", math.radians(30)) 
    side = kwargs.get("side", "front")
    threshold = kwargs.get("threshold", 2)
    world_state = realization.get_world_state(step)
    ego_state = world_state.ego_state
    candidates = proximity_filter(world_state.other_vehicle_states, ego_state, kwargs.get("proximity", 10))
    if len(candidates) == 0:
        return 0
    
    ego_yaw = ego_state.orientation.yaw
    ego_heading = Vector(math.cos(ego_yaw), math.sin(ego_yaw), 0).normalized()
    violation = 0
    
    for object_state in world_state.other_vehicle_states:
        ego_to_object = (object_state.position - ego_state.position).normalized()
        angle = ego_heading.angleWith(ego_to_object)
        if abs(angle) <= front_angle/2:
            object_side = "front"
        elif angle < 0:
            object_side = "right"
        else:
            object_side = "left"

        if side == object_side:
            violation = max(violation, threshold - polygon_distance(ego_state, object_state), 0)
    return violation



def front_clearance_vector(realization, step, **kwargs):
    return clearance_vector_based(realization, step, side="front", **kwargs)

f11_c = rule_function(front_clearance_vector, max)

def left_clearance_vector(realization, step, **kwargs):
    return clearance_vector_based(realization, step, side="left", **kwargs)

f12_c = rule_function(left_clearance_vector, max)

def right_clearance_vector(realization, step, **kwargs):
    return clearance_vector_based(realization, step, side="right", **kwargs)

f13_c = rule_function(right_clearance_vector, max)
'''







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


'''
def rule_vru_collision(realization, step, car_mass=1500, vru_mass=70, momentum = False):
    ego = realization.ego
    ego_state = ego.get_state(step)
    ego_polygon = ego_state.polygon
    ego_velocity = (ego_state.velocity[0], ego_state.velocity[1])
    
    objects = realization.vrus
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
            
            if momentum:
                current_violation = car_mass * (np.linalg.norm(ego_velocity) - np.linalg.norm(ego_after_velocity)) + \
                                    vru_mass * (np.linalg.norm(obj_velocity) - np.linalg.norm(obj_after_velocity))
            else:
                current_violation = 0.5 * car_mass * (np.linalg.norm(ego_velocity) ** 2 - np.linalg.norm(ego_after_velocity) ** 2) + \
                                0.5 * vru_mass * (np.linalg.norm(obj_after_velocity) ** 2 - np.linalg.norm(obj_velocity) ** 2)
            violation = max(violation, current_violation)
            break
    
    return violation

def rule_vehicle_collision(realization, step, car_mass=1500, momentum = False):
    ego = realization.ego
    ego_state = ego.get_state(step)
    ego_polygon = ego_state.polygon
    ego_velocity = (ego_state.velocity[0], ego_state.velocity[1])
    
    objects = realization.other_vehicles
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

            if momentum:
                current_violation = car_mass * (np.linalg.norm(ego_velocity) - np.linalg.norm(ego_after_velocity)) + \
                                    car_mass * (np.linalg.norm(obj_velocity) - np.linalg.norm(obj_after_velocity))
            else:
                current_violation = 0.5 * car_mass * (np.linalg.norm(ego_velocity) ** 2 - np.linalg.norm(ego_after_velocity) ** 2) + \
                                    0.5 * car_mass * (np.linalg.norm(obj_velocity) ** 2 - np.linalg.norm(obj_after_velocity) ** 2)
            violation = max(violation, current_violation)
            break
    
    return violation

def collision_modified(realization, step, object_type="VRU", car_mass=1500, vru_mass=70, momentum=False):
    if object_type == "Vehicle":
        mass = car_mass
        objects = realization.vrus
    elif object_type == "VRU":
        mass = vru_mass
        objects = realization.other_vehicles
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
            
        if step == 0:
            ego_before = ego.get_state(step)
            obj_before = obj.get_state(step)
        else:
            ego_before = ego.get_state(step-1) # state before collision
            obj_before = obj.get_state(step-1)
        
        
        
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
            
            if momentum:
                momentum_before = mass * ego_before.velocity + mass * obj_before.velocity
                momentum_after = mass * next_ego_state.velocity + mass * next_obj_state.velocity
                current_violation = momentum_before - momentum_after
            else:
                kinetic_energy_before = 0.5 * mass * (ego_before.velocity.norm()) + 0.5 * mass * (obj_before.velocity.norm())
                kinetic_energy_after = 0.5 * mass * (next_ego_state.velocity.norm() ** 2) + 0.5 * mass * (next_obj_state.velocity.norm() ** 2)
                current_violation = kinetic_energy_before - kinetic_energy_after
            violation = max(violation, current_violation)
            
    return violation
            

def vru_collision(realization, step, **kwargs):
    return collision_modified(realization, step, object_type="VRU", **kwargs)

def vru_collision_momentum(realization, step, **kwargs):
    return collision_modified(realization, step, object_type="VRU", momentum=True, **kwargs)



f1 = rule_function(rule_vru_collision, max)             

f1_b = rule_function(vru_collision_momentum, max)

def vehicle_collision(realization, step, **kwargs):
    return collision_modified(realization, step, object_type="Vehicle", **kwargs)

f2 = rule_function(rule_vehicle_collision, max)

def vehicle_collision_momentum(realization, step, **kwargs):
    return collision_modified(realization, step, object_type="Vehicle", momentum=True, **kwargs)

f2_b = rule_function(vehicle_collision_momentum, max)
'''

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


'''

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
    
    objects = [obj for obj in realization.other_objects]
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

def vehicle_time_to_collision(realization, step, threshold=0.8, step_size=0.05):
    
    def check_intersection(ego_polygon, adv_polygon, ego_velocity, adv_velocity, step_size=step_size, threshold_time=threshold):
        ego_vx, ego_vy = ego_velocity 
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

    objects = realization.other_objects
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

'''


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



def vru_acknowledgement(realization, step, **kwargs):
    proximity = kwargs.get("proximity", 2)
    threshold = kwargs.get("threshold", 0)
    timesteps = kwargs.get("timesteps", 30)
    ego = realization.ego
    vrus = realization.vrus
    
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
                
                relative_velocity_before = (float(ego_before_velocity.dot(relative_position)) * relative_position).norm()
                relative_velocity_after = (float(ego_after_velocity.dot(relative_position)) * relative_position).norm()

                acceleration = (relative_velocity_after - relative_velocity_before) / realization.delta
                violation = max(violation, acceleration - threshold)
                break
    
    return violation
        
    '''