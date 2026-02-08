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
from rulebook_benchmark.rulebook import Relation


class Result:
    def __init__(self, minimum_violation=0, aggregation_method=max):
        self.total_violation = minimum_violation
        self.violation_history = []
        self.aggregation_method = aggregation_method
    def add(self, violation):
        self.total_violation = self.aggregation_method((self.total_violation, violation))
        self.violation_history.append(self.total_violation)




class Rule:
    def __init__(self, calculate_violation, aggregation_method, name, rule_id, **kwargs):
        self.calculate_violation = calculate_violation
        self.aggregation_method = aggregation_method
        self.parameters = kwargs
        self.name = name
        self.id = rule_id

    def __call__(self, handler, step, **runtime_params):
        # merge init parameters and runtime ones
        params = {**self.parameters, **runtime_params}
        return self.calculate_violation(handler, step, **params)
    
    def copy(self):
        return Rule(self.calculate_violation, self.aggregation_method, self.name, self.id, **self.parameters)

    def evaluate(self, handler, **runtime_params):
        result = Result(aggregation_method=self.aggregation_method)
        for step in range(handler.max_steps):
            result.add(self(handler, step, **runtime_params))
        return result.total_violation
    
    
    def evaluate_with_cache(self, handler, rule_parameter_result_dict, scenario, rule_id, **runtime_params):
        params = self.parameters
        param_tuple = tuple(sorted(params.items())) if params else ()
        
        if rule_id in rule_parameter_result_dict:
            pass
        else:
            rule_parameter_result_dict[rule_id] = {}
            
        if param_tuple in rule_parameter_result_dict[rule_id]:
            pass
        else:
            rule_parameter_result_dict[rule_id][param_tuple] = {}
            
        if scenario in rule_parameter_result_dict[rule_id][param_tuple]:
            return rule_parameter_result_dict[rule_id][param_tuple][scenario]
        
        result = Result(aggregation_method=self.aggregation_method)
        for step in range(handler.max_steps):
            result.add(self(handler, step, **runtime_params))
        
        rule_parameter_result_dict[rule_id][param_tuple][scenario] = result.total_violation
        return result.total_violation
        



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


    def evaluate_with_cache(self, rule_parameter_result_dict, scenario):
        rule_id_to_params = {}

        for name, rule in self.rules.items():
            params = rule.parameters
            rule_id_to_params[name] = tuple(sorted(params.items())) if params else ()
        

        # initialize results per rule
        results = {}

        
        cached = set()
        for name in self.rules.keys():
            if name in rule_parameter_result_dict:
                pass
            else:
                rule_parameter_result_dict[name] = {}
                
            if rule_id_to_params[name] in rule_parameter_result_dict[name]:
                pass
            else:
                rule_parameter_result_dict[name][rule_id_to_params[name]] = {}
                
            if scenario in rule_parameter_result_dict[name][rule_id_to_params[name]]:
                violation_score = rule_parameter_result_dict[name][rule_id_to_params[name]][scenario]
                results[name] = violation_score
                cached.add(name)
            else:
                for d in rule_parameter_result_dict[name]:
                    print(d)
                pass
        return results


def kinetic_energy_loss(ego_velocity_before, ego_velocity_after, adv_velocity_before, adv_velocity_after, ego_mass, adv_mass, VRU=False):
    ego_loss = 0.5 * ego_mass * (np.linalg.norm(ego_velocity_before) ** 2 - np.linalg.norm(ego_velocity_after) ** 2)
    adv_loss = 0.5 * adv_mass * (np.linalg.norm(adv_velocity_before) ** 2 - np.linalg.norm(adv_velocity_after) ** 2)
    
    if VRU:
        # If VRU is involved, we check how much kinetic energy the ego lost, and how much the VRU gained
        adv_loss = -adv_loss
        
    return ego_loss + adv_loss

def momentum_loss(ego_velocity_before, ego_velocity_after, adv_velocity_before, adv_velocity_after, ego_mass, adv_mass):
    ego_momentum_loss = np.linalg.norm(ego_mass * (ego_velocity_after - ego_velocity_before))
    adv_momentum_loss = np.linalg.norm(adv_mass * (adv_velocity_after - adv_velocity_before))

    return ego_momentum_loss + adv_momentum_loss

def generalized_collision(handler, collision_timeline, states, step, ego_mass, adv_mass, momentum, epsilon=1e-6, VRU=False):
    violation = 0
    for state in states:
        uid = state.uid
        if uid not in collision_timeline:
            continue
        collisions = collision_timeline[uid]
        for collision in collisions:
            collision_start, collision_end = collision
            if collision_start == 0:
                before_collision = collision_start
            else:
                before_collision = collision_start - 1
            if before_collision > step:
                break
            elif before_collision < step:
                continue
            else:
                prev_state = handler(before_collision).ego_state
                after_state = handler(collision_end).ego_state

                adv_prev_state = handler(before_collision).world_state[uid]
                adv_after_state = handler(collision_end).world_state[uid]

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
                        adv_mass=adv_mass,
                        VRU=VRU
                    ))

                curr_violation = max(curr_violation, epsilon) # ensure non-zero violation for any collision
                violation += curr_violation

    return violation


def vru_collision(handler, step, car_mass=1500, vru_mass=70, momentum = False):
    vru_states = handler(step).vrus_in_proximity
    return generalized_collision(handler, handler.collision_timeline, vru_states, step, car_mass, vru_mass, momentum, VRU=True)


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

    ego_vel_normalized = normalize_vector(ego_vel)
    ego_to_adv = normalize_vector(adv_pos - ego_pos)
    projection = np.dot(ego_vel_normalized, ego_to_adv)        
    # if intersection happens after horizon, skip expensive TTC
    if t > horizon or u > horizon or t < 0 or u < 0 or projection <= 0:
        return False

    return True  # possible interaction, run continuous_ttc

    
    



def vru_ttc(handler, step, threshold=1.0):
    pool = handler(step)
    ego_state = pool.ego_state
    ego_velocity = ego_state.velocity
    ego_position = ego_state.position
    ego_polygon = ego_state.polygon.convex_hull # comment/uncomment
    
    violation = 0
    
    for state in pool.vru_states:
        obj_velocity = state.velocity
        obj_pos = state.position
        if not early_ttc(ego_position, ego_velocity, obj_pos, obj_velocity, threshold):
            continue
        
        v_rel = (obj_velocity[0] - ego_velocity[0], obj_velocity[1] - ego_velocity[1])
        ttc = continuous_ttc(ego_polygon.exterior.coords[:-1], state.polygon.convex_hull.exterior.coords[:-1], v_rel, threshold)
        if ttc is not None:
            violation = max(violation, threshold - ttc)

    return violation


def vehicle_ttc(handler, step, threshold=0.8):
    pool = handler(step)
    ego_state = pool.ego_state
    ego_velocity = ego_state.velocity
    ego_position = ego_state.position
    ego_polygon = ego_state.polygon
    ego_polygon = ego_polygon.convex_hull # comment/uncomment
    violation = 0
    
    for state in pool.other_vehicle_states:

        obj_velocity = state.velocity
        obj_polygon = state.polygon
        obj_polygon = obj_polygon.convex_hull # comment/uncomment
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


def vru_clearance(handler, step, on_road, threshold):
    pool = handler(step)
    vru_states = pool.vrus_in_proximity
    violation = 0
    distance = np.inf
    for vru_state in vru_states:
        if on_road and vru_state.lane is not None:
            distance = pool.distance(vru_state)
        elif not on_road and vru_state.lane is None:
            distance = pool.distance(vru_state)

        violation = max(violation, threshold - distance)

    return violation


f8 = Rule(vru_clearance, max, on_road=False, threshold=1)
f9 = Rule(vru_clearance, max, on_road=True, threshold=1)

    
def vru_acknowledgement(handler, step, threshold = 0, timesteps = 20, velocity = 4):
    candidates = set()
    violation = 0
    num_vrus = len(handler.vru_uids)
    for i in range(step, min(step + timesteps, len(handler.realization))):
        pool = handler(i)
        vrus = pool.vrus_in_proximity
        for vru_state in vrus:
            candidates.add(vru_state.uid)
        if len(candidates) == num_vrus:
            break


    pool = handler(step)
    ego_acceleration = pool.ego_state.acceleration
    for uid in candidates:
        state = pool.world_state[uid]
        relative_position = state.position - pool.ego_state.position
        relative_position = normalize_vector(relative_position)
        # first check if projected velocity towards ego is above threshold
        projected_velocity = np.dot(state.velocity, relative_position)
        if projected_velocity > velocity:
            ego_acceleration_projected = np.dot(ego_acceleration, relative_position)
            violation = max(0, ego_acceleration_projected - threshold, violation)
        else:
            continue
    return violation

f5 = Rule(vru_acknowledgement, max, threshold = -1, timesteps = 30, velocity = 4)
# TODO: vehicle yielding rule based on adv vehicle decelerations

def correct_side(handler, step, relax_at_intersections=False): # use relax_at_intersections if your lane polygons do not cover all correct sides at intersections
    ego_state = handler(step).ego_state
    
    correct_area = shapely.Polygon()
    incorrect_area = shapely.Polygon()
    
    for lane in ego_state.correct_lanes:
        lane_polygon = lane.polygon
        if handler.realization.network.intersectionAt(ego_state.position) is not None and relax_at_intersections:
            #lane_polygon = lane.polygon.buffer(0.8)  # allow some buffer at intersections
            return 0 # if at intersection, touching correct lane is enough
        correct_area = correct_area.union(lane_polygon)

    for lane in ego_state.incorrect_lanes:
        incorrect_area = incorrect_area.union(lane.polygon)
    
    pure_incorrect_area = incorrect_area.difference(correct_area)
    ego_polygon = ego_state.polygon
    
    ego_violation_area = ego_polygon.intersection(pure_incorrect_area).area

    return ego_violation_area

def correct_side_alt(handler, step, relax_at_intersections=False, fine_grained=True): # use relax_at_intersections if your lane polygons do not cover all correct sides at intersections
    ego_state = handler(step).ego_state

    isScenic = handler.realization.isScenic
    rot = 0
    ego_lane = ego_state.lane

    if isScenic:
        rot = np.pi/2

    if ego_lane is None or handler.realization.network.intersectionAt(ego_state.position) is not None and relax_at_intersections:
        return 0

    ego_lane_heading = ego_lane.orientation.value(ego_state.position) + rot

    if math.cos(ego_lane_heading - ego_state.orientation.yaw) < 0:
        if fine_grained:
            return shapely.intersection(ego_state.polygon, ego_lane.polygon).area
        return 1
    else:
        return 0

f7_alt = Rule(correct_side_alt, sum, relax_at_intersections=True, fine_grained=True)

f7 = Rule(correct_side, sum, relax_at_intersections=True)

def speed_limit(handler, step, threshold=15): # speed limit
    ego_state = handler(step).ego_state
    ego_velocity = norm(ego_state.velocity)
    if ego_state.lane is None or ego_state.lane.speedLimit is None:
        speed_limit = threshold
    else:
        speed_limit = ego_state.lane.speedLimit
    
    return max(0, ego_velocity - speed_limit)**2

f15 = Rule(speed_limit, max, threshold=15)

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

f11_sum = Rule(front_clearance, sum, threshold=0.8)
f12_sum = Rule(side_clearance, sum, left=True, threshold=0.8)
f13_sum = Rule(side_clearance, sum, left=False, threshold=0.8)






def clearance_vector_based(handler, step, threshold=0.8, side_angle=90, side="front"):
    side_angle = math.radians(side_angle)
    pool = handler(step)
    ego_state = pool.ego_state
    states = pool.vehicles_in_proximity
    
    violation = 0
    
    ego_heading_vector = normalize_vector(np.array([math.cos(ego_state.orientation.yaw), math.sin(ego_state.orientation.yaw)]))
    
    for state in states:
        state_vector = normalize_vector(np.array([math.cos(state.orientation.yaw), math.sin(state.orientation.yaw)]))
        angle = angle_between(ego_heading_vector, state_vector)
        if (abs(angle) <= side_angle / 2 and side == "front") or (-side_angle * 3/2 < angle < -side_angle/2 and side == "right") or (side_angle * 3/2 > angle > side_angle/2 and side == "left"):
            violation = max(violation, threshold - pool.distance(state))

    return violation

f11_v = Rule(clearance_vector_based, max, threshold=0.8, side_angle=90, side="front")
f12_v = Rule(clearance_vector_based, max, side="left", threshold=0.8, side_angle=90)
f13_v = Rule(clearance_vector_based, max, side="right", threshold=0.8, side_angle=90)
