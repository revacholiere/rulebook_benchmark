from trimesh.transformations import compose_matrix
from scenic.core.regions import MeshVolumeRegion, EmptyRegion
import shapely
from rulebook_benchmark.realization import Realization
import math
from scenic.core.vectors import Vector


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


            
def rule_11(realization, step, threshold=5): # front vehicle clearance
    ego = realization.get_ego()
    ego_state = realization.get_ego_state(step)
    ego_lane = realization.network.laneAt(ego_state.position)
    ego_polygon = ego.get_polygon(ego_state)
    
    if len(realization.vehicles) == 1 or ego_lane is None: #base case for no vehicles or ego not in drivable area
        return 0
    
    proximity_threshold = threshold*3
    objects = realization.vehicles[1:]
    candidate_objects = []
    candidate_distances = []
    
    for obj in objects: # filter out far objects
        obj_state = obj.get_state(step)
        obj_polygon = obj.get_polygon(obj_state)
        dist = shapely.distance(ego_polygon, obj_polygon)
        if dist < proximity_threshold:
            candidate_objects.append(obj)
            candidate_distances.append(dist)
            
    if len(candidate_objects) == 0: # no objects in proximity
        return 0
    
    network = realization.network
    intersection = network.intersectionAt(ego_state.position)
    
    max_violation = 0
    possible_lanes = []
    next_lanes = []
    if intersection is None:
        if ego_lane.maneuvers[0].intersection is not None: # ego has maneuvers in an intersection
            for maneuver in ego_lane.maneuvers:
                next_lanes.append(maneuver.connectingLane)
        else:
            assert len(ego_lane.maneuvers) == 1
            next_lanes.append(ego_lane.successor)
            possible_lanes.append(ego_lane)

    else: # ego in intersection
        assert ego_lane.predecessor is not None
        predecessor = ego_lane.predecessor
        for maneuver in predecessor.maneuvers:
            possible_lanes.append(maneuver.connectingLane)
            next_lanes.append(maneuver.endLane)
        
    ego_point = shapely.Point(ego_state.position.x, ego_state.position.y)
        
    for obj, dist in zip(candidate_objects, candidate_distances):
        obj_state = obj.get_state(step)
        obj_polygon = obj.get_polygon(obj_state)
        for possible_lane, next_lane in zip(possible_lanes, next_lanes):
            lane_polygon = possible_lane.polygons
            if shapely.intersects(lane_polygon, obj_polygon) and shapely.intersects(lane_polygon, ego_point):
                centerline = ego_lane.centerline.lineString.segmentize(1).coords[:]
                min_ego_dist = float("inf")
                min_obj_dist = float("inf")
                ego_ind = 0
                obj_ind = 0
                for p in range(len(centerline)):
                    point = centerline[p]
                    point = Vector(point[0], point[1], 0)
                    ego_dist = (point - ego_state.position).norm()
                    obj_dist = (point - obj_state.position).norm()

                    if ego_dist < min_ego_dist:
                        ego_ind = p
                    if obj_dist < min_obj_dist:
                        min_obj_dist = obj_dist
                        obj_ind = p

                if ego_ind < obj_ind:
                    max_violation = max(max_violation, threshold - dist)
                break
            next_lane_polygon = next_lane.polygons
            if shapely.intersects(next_lane_polygon, obj_polygon) and shapely.intersects(lane_polygon, ego_point):
                max_violation = max(max_violation, threshold - dist)
                break 
                    
                    
                    
                
                    
                
    return max_violation
            
            
def rule_12(realization, step, threshold=2): # left vehicle clearance
    ego = realization.get_ego()
    net = realization.network
    ego_state = realization.get_ego_state(step)
    

    ego_lane_group = net.laneGroupAt(ego_state.position)
    ego_lane = net.laneAt(ego_state.position)
    
    if ego_lane_group is None or ego_lane is None: # ego not in drivable area
        return 0
    
    adjacent_lanes = ego_lane_group.adjacentLanes
    if len(adjacent_lanes) == 1 and adjacent_lanes[0] not in ego_lane_group.lanes or len(adjacent_lanes) == 2:
        adv_lane = adjacent_lanes[0]
    # add one more clause checking lane group
    else:
        return 0
    
    objects = realization.vehicles[1:]
    distances = []
    for obj in objects: # find closest object in adjacent left lane
        obj_state = obj.get_state(step)
        obj_lane = net.laneAt(obj_state.position)
        if obj_lane is None or obj_lane != adv_lane:
            continue
        distance = (objects[obj].position - ego_state.position).norm()
        distances.append(distance)
    if len(distances) == 0:
        return 0
    ind = distances.index(min(distances))
    obj = objects[ind]
    obj_state = obj.get_state(step)
    obj_region = MeshVolumeRegion(mesh=obj.mesh, dimensions=obj.dimensions, position=obj_state.position, rotation=obj_state.orientation)
    obj_polygon = obj_region.boundingPolygon.polygons
    ego_region = MeshVolumeRegion(mesh=ego.mesh, dimensions=ego.dimensions, position=ego_state.position, rotation=ego_state.orientation)
    ego_polygon = ego_region.boundingPolygon.polygons
    distance = ego_polygon.distance(obj_polygon)
    return max(0, distance - threshold)
        
        
        
def rule_13(realization, step, threshold=2): # right vehicle clearance
    pass

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