import shapely

def isObjectInLane(state, lane): # check if the object's center is in the lane
    lane_polygon = lane.polygon
    object_point = shapely.Point(state.position.x, state.position.y)
    return lane_polygon.contains(object_point)

def firstPass(obj, lanes): # process the states where the lane is not ambiguous
    possible_lanes = {}
    ambiguous_lanes = {}
    for i in range(len(obj.trajectory)):
        state = obj.get_state(i)
        possible_lanes[i] = []
        for lane in lanes:
            if isObjectInLane(state, lane):
                possible_lanes[i].append(lane)
                

        if len(possible_lanes[i]) == 1:
            obj.trajectory[i].lane = possible_lanes[i][0]
        else: # needs second pass
            ambiguous_lanes[i] = possible_lanes[i]
                   
    return ambiguous_lanes
        
def secondPass(obj, ambiguous_lanes): # process the states where the lane is ambiguous, by checking previous and next states
    for idx, lanes in ambiguous_lanes.items():
        if idx > 0:
            prev_lane = obj.trajectory[idx - 1].lane
            for lane in lanes:
                if lane == prev_lane: # if one of the ambiguous lanes is the same as the previous lane, assign it
                    obj.trajectory[idx].lane = lane
                    continue
                if len(prev_lane.maneuvers) == 1: # one way to go from previous lane
                    end_lane = prev_lane.maneuvers[0].endLane
                    if end_lane == lane:
                        obj.trajectory[idx].lane = lane
                        continue
            
        j = idx + 1
        while j < len(obj.trajectory) and obj.trajectory[j].lane is None:
            j += 1
            
        if j < len(obj.trajectory): # non-ambiguous lane found in the future
            for lane in lanes:
                for maneuver in lane.maneuvers:
                    end_lane = maneuver.endLane
                    future_lane = obj.get_state(j).lane
                    if end_lane == future_lane:
                        obj.trajectory[idx].lane = lane
                        continue
        else: # no non-ambiguous lane found in the future, then find the lane with the closest orientation
            angles = []
            for lane in lanes:
                lane_orientation = lane.orientation.value(obj.get_state(idx).position)
                angles.append(abs(lane_orientation - obj.get_state(idx).orientation))
            min_idx = angles.index(min(angles))
            obj.trajectory[idx].lane = lanes[min_idx]
            
            
        

def process_trajectory(realization): # given a realization, extract the sequence of lanes followed by each vehicle
    network = realization.network
    objects = realization.vehicles
    
    lanes = network.lanes
        

    for obj in objects: 
        ambiguous_lanes = firstPass(obj, lanes)
        if len(ambiguous_lanes) > 0: 
            secondPass(obj, ambiguous_lanes)
            
        print(f"Object {obj.object_type} {obj.mesh} has trajectory: {[state.lane for state in obj.trajectory]}")

    
    
    
    