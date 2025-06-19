import shapely


def isObjectInLane(state, lane):  # check if the object's center is in the lane
    lane_polygon = lane.polygon
    object_point = shapely.Point(state.position.x, state.position.y)
    return lane_polygon.contains(object_point)


def firstPass(obj, lanes):  # process the states where the lane is not ambiguous
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
        elif len(possible_lanes[i]) == 0:  # out of road
            obj.trajectory[i].lane = None
        else:  # needs second pass
            ambiguous_lanes[i] = possible_lanes[i]

    print(len(ambiguous_lanes), "states with ambiguous lanes")
    return ambiguous_lanes


def secondPass(obj, ambiguous_lanes, network):
    for step, lanes in ambiguous_lanes.items():
        pos = obj.get_state(step).position

        intersection = network.intersectionAt(pos)

        found = False
        if intersection is not None:
            if step > 0:
                prev_lane = obj.get_state(step - 1).lane
                for lane in lanes:
                    if prev_lane == lane:
                        obj.trajectory[step].lane = lane
                        found = True
                        #print("found same lane as previous", step)
                        break
                if found:
                    continue

            j = step + 1
            while j < len(obj.trajectory) and obj.trajectory[j].lane not in intersection.outgoingLanes:
                j += 1

            candidate_lanes = []
            future_lane = None
            if j < len(obj.trajectory): future_lane = obj.get_state(j).lane

            if future_lane is not None:  # non-ambiguous lane or lane group found in the future
                future_lane = obj.get_state(j).lane
                for lane in lanes:
                    end_lane = lane.successor
                    if end_lane == future_lane:
                        candidate_lanes.append(lane)
                    elif end_lane.group == future_lane.group:
                        candidate_lanes.append(lane)

                if len(candidate_lanes) == 1:
                    obj.trajectory[step].lane = candidate_lanes.pop()
                    found = True
                    continue

            k = step - 1
            while k >= 0 and obj.trajectory[k].lane not in intersection.incomingLanes:
                k -= 1

            candidate_lanes_2 = []
            prev_lane = None
            if k >= 0: prev_lane = obj.get_state(k).lane

            if prev_lane is not None and future_lane is not None:  # non-ambiguous lane or lane group found in the future and past
                for lane in candidate_lanes:
                    end_lane = lane.successor
                    start_lane = lane.predecessor

                    if end_lane == future_lane and start_lane == prev_lane:
                        candidate_lanes_2.append(lane)
                    elif (
                        end_lane.group == future_lane.group
                        and start_lane.group == prev_lane.group
                    ):
                        candidate_lanes_2.append(lane)
                if len(candidate_lanes_2) == 1:
                    obj.trajectory[step].lane = candidate_lanes_2.pop()
                    found = True
                    continue
                else:
                    for lane in candidate_lanes_2:
                        print(lane.id)

            angles = []

            if len(candidate_lanes_2) > 1:
                last_resort = candidate_lanes_2
            elif len(candidate_lanes) > 1:
                last_resort = candidate_lanes
            else:
                last_resort = lanes

            for lane in last_resort:  # workaround for when no candidate lanes are found
                lane_orientation = lane.orientation.value(obj.get_state(step).position)
                angles.append(
                    abs(lane_orientation - obj.get_state(step).orientation.yaw)
                )
            min_idx = angles.index(min(angles))
            obj.trajectory[step].lane = lanes[min_idx]
        else:
            prev_lane = obj.get_state(step - 1).lane if step > 0 else None
            if prev_lane is not None:
                for lane in lanes:
                    if prev_lane == lane or lane in [maneuver.endLane for maneuver in prev_lane.maneuvers]:
                        obj.trajectory[step].lane = lane
                        found = True
                        #print("found same lane as previous", step)
                        break
            else:
                angles = []
                for lane in lanes:
                    lane_orientation = lane.orientation.value(obj.get_state(step).position)
                    angles.append(
                        abs(lane_orientation - obj.get_state(step).orientation.yaw)
                    )
                min_idx = angles.index(min(angles))
                obj.trajectory[step].lane = lanes[min_idx]
                    
            #print("found lane with closest orientation", step)


def process_trajectory(
    realization,
):  # given a realization, extract the sequence of lanes followed by each vehicle
    network = realization.network
    objects = realization.vehicles

    lanes = network.lanes

    for obj in objects:
        ambiguous_lanes = firstPass(obj, lanes)
        if len(ambiguous_lanes) > 0:
            secondPass(obj, ambiguous_lanes, network)

        # print(f"Object {obj.object_type} {obj.mesh} has trajectory: {[state.lane for state in obj.trajectory]}")


def process_trajectory_old(realization):
    network = realization.network
    
    objects = realization.vehicles
    for obj in objects:
        for i in range(len(obj.trajectory)):
            state = obj.get_state(i)
            state.lane = network.laneAt(state.position)
            #if state.lane is None:
            #    print(f"Object {obj.object_type} {obj.mesh} is out of road at step {i}")