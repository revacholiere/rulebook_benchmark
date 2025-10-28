from enum import Enum
import json
import random

AgentType = Enum('AgentType', 'CAR PEDESTRIAN')
VehicleManeuver = Enum('VehicleManeuver', 'STRAIGHT LEFT_TURN RIGHT_TURN LANE_CHANGE LANE_FOLLOWING') # U_TURN
PedestrianManeuver = Enum('PedestrianManeuver', 'CROSS_STREET WALK_ALONG_SIDEWALK')
SpatialRelation = Enum('SpatialRelation', 'AHEAD_OF BEHIND FASTER_LANE SLOWER_LANE OPPOSITE_INTERSECTION LATERAL_INTERSECTION') # OPPOSITE_LANE
PedestrianSpatialRelation = Enum('PedestrianSpatialRelation', 'SIDEWALK') # TR_INTER TL_INTER BR_INTER BL_INTER
ParamToRange = {'INIT_DIST': (10, 20), 'SAFETY_DIST': (6, 10), 'BYPASS_DIST': (10, 12), 'SPEED': (6, 11), 'BRAKE': (0.5, 1.0), 'PED_LONGITUDINAL_OFFSET': (5, 15), 'PED_SPEED': (1, 3)}
ConstantToRange = {'INTER_DIST': [15, 25], 'INIT_DIST': 10, 'PED_LATERAL_OFFSET': 8, 'PED_LONGITUDINAL_OFFSET': 30}

parameter_tracker = {} # dict of parameter name to (low, high)
constant_tracker = {} # dict of constant name to value
requirement_tracker = [] # list of requirements

def scenario_spec_checker(spec):
    # Check that ego is specified
    assert 'ego' in spec, "Ego agent must be specified in the scenario spec."
    
    # Check that maneuvers are valid:
    ego_maneuver = spec['ego']['maneuver']
    if ego_maneuver not in VehicleManeuver:
        raise ValueError(f"Invalid ego maneuver: {ego_maneuver}")
    for agent_name, agent_spec in spec.get('agents', {}).items():
        maneuver = agent_spec['maneuver']
        if agent_spec['type'] == AgentType.CAR:
            if maneuver not in VehicleManeuver:
                raise ValueError(f"Invalid maneuver for agent {agent_name} of type CAR: {maneuver}")
        elif agent_spec['type'] == AgentType.PEDESTRIAN:
            if maneuver not in PedestrianManeuver:
                raise ValueError(f"Invalid maneuver for agent {agent_name} of type PEDESTRIAN: {maneuver}")
    
    # Check that spatial relations are valid:
    for agent_name, agent_spec in spec.get('agents', {}).items():
        spatial_relation = agent_spec['spatial_relation']
        if agent_spec['type'] == AgentType.CAR:
            if spatial_relation not in SpatialRelation:
                raise ValueError(f"Invalid spatial relation for agent {agent_name} of type CAR: {spatial_relation}")
        elif agent_spec['type'] == AgentType.PEDESTRIAN:
            if spatial_relation not in PedestrianSpatialRelation:
                raise ValueError(f"Invalid spatial relation for agent {agent_name} of type PEDESTRIAN: {spatial_relation}")
    
    # Spatial constraints: faster lane and slower lane cannot both exist
    faster_lane_exists = any(agent_spec['spatial_relation'] == SpatialRelation.FASTER_LANE for agent_spec in spec.get('agents', {}).values())
    slower_lane_exists = any(agent_spec['spatial_relation'] == SpatialRelation.SLOWER_LANE for agent_spec in spec.get('agents', {}).values())
    if faster_lane_exists and slower_lane_exists:
        raise ValueError("Scenario cannot have both FASTER_LANE and SLOWER_LANE spatial relations among agents.")
    
    # Spatial constaints based on maneuvers: Assume left lane is slower lane, right lane is faster lane
    ego_maneuver = spec['ego']['maneuver']
    faster_lane_agents_existed = False
    slower_lane_agents_existed = False
    for agent_name, agent_spec in spec.get('agents', {}).items():
        spatial_relation = agent_spec['spatial_relation']
        maneuver = agent_spec['maneuver']
        if spatial_relation == SpatialRelation.SLOWER_LANE and maneuver == VehicleManeuver.LEFT_TURN:
            raise ValueError(f"Agent {agent_name} cannot have spatial relation SLOWER_LANE while performing LEFT_TURN maneuver.")
        if spatial_relation == SpatialRelation.FASTER_LANE and maneuver == VehicleManeuver.RIGHT_TURN:
            raise ValueError(f"Agent {agent_name} cannot have spatial relation FASTER_LANE while performing RIGHT_TURN maneuver.")
        if spatial_relation == SpatialRelation.FASTER_LANE and ego_maneuver == VehicleManeuver.LEFT_TURN:
            raise ValueError(f"Agent {agent_name} cannot have spatial relation FASTER_LANE while ego is performing LEFT_TURN maneuver.")
        if spatial_relation == SpatialRelation.SLOWER_LANE and ego_maneuver == VehicleManeuver.RIGHT_TURN:
            raise ValueError(f"Agent {agent_name} cannot have spatial relation SLOWER_LANE while ego is performing RIGHT_TURN maneuver.")
        if spatial_relation == SpatialRelation.FASTER_LANE:
            faster_lane_agents_existed = True
        if spatial_relation == SpatialRelation.SLOWER_LANE:
            slower_lane_agents_existed = True
    if faster_lane_agents_existed:
        for agent_name, agent_spec in spec.get('agents', {}).items():
            spatial_relation = agent_spec['spatial_relation']
            maneuver = agent_spec['maneuver']
            if spatial_relation in [SpatialRelation.AHEAD_OF, SpatialRelation.BEHIND] and maneuver == VehicleManeuver.LEFT_TURN:
                raise ValueError(f"Agent {agent_name} cannot have spatial relation {spatial_relation.name} while performing LEFT_TURN maneuver when other agents are in FASTER_LANE.")
    if slower_lane_agents_existed:
        for agent_name, agent_spec in spec.get('agents', {}).items():
            spatial_relation = agent_spec['spatial_relation']
            maneuver = agent_spec['maneuver']
            if spatial_relation in [SpatialRelation.AHEAD_OF, SpatialRelation.BEHIND] and maneuver == VehicleManeuver.RIGHT_TURN:
                raise ValueError(f"Agent {agent_name} cannot have spatial relation {spatial_relation.name} while performing RIGHT_TURN maneuver when other agents are in SLOWER_LANE.")

def scenario_generator(spec):
    """Given a scenario specification, generate the corresponding Scenic scenario file.

    Args:
        spec (dict): Scenario specification dictionary.
    """
    f = open(spec['scenario'], 'w')
    
    f.write(_title_generator(spec))
    f.write(_model_generator(spec))
    
    # Need to collect parameter and constant info first
    spatial_code = _spatial_generator(spec)
    behavior_code = _behavior_generator(spec)
    spec_code = _specification_generator(spec)
    
    f.write(_constant_generator(spec))
    f.write(spatial_code)
    f.write(behavior_code)
    f.write(spec_code)
    f.write(_recording_generator(spec))
    
    f.close()
    
def _title_generator(spec):
    code = ""
    code += f'"""\n'
    code += f"SCENARIO: {spec['scenario']}\n"
    code += f"DESCRIPTION: "
    ego_spec = spec['ego']
    if ego_spec['maneuver'] == VehicleManeuver.STRAIGHT:
        code += f"Ego vehicle goes straight in the intersection. "
    elif ego_spec['maneuver'] in [VehicleManeuver.LEFT_TURN, VehicleManeuver.RIGHT_TURN]:
        code += f"Ego vehicle makes a {ego_spec['maneuver'].name.replace('_', ' ').lower()} in the intersection. "
    elif ego_spec['maneuver'] == VehicleManeuver.LANE_FOLLOWING:
        code += f"Ego vehicle follows its lane. "
    elif ego_spec['maneuver'] == VehicleManeuver.LANE_CHANGE:
        code += f"Ego vehicle changes lane when possible. "
    for agent_name, agent_spec in spec.get('agents', {}).items():
        code += f"{agent_name} is a {agent_spec['type'].name.lower()} that starts "
        if agent_spec['spatial_relation'] in [SpatialRelation.AHEAD_OF, SpatialRelation.BEHIND]:
            code += f"{agent_spec['spatial_relation'].name.replace('_', ' ').lower()} the ego vehicle and "
        elif agent_spec['spatial_relation'] in [SpatialRelation.FASTER_LANE, SpatialRelation.SLOWER_LANE]:
            code += f"in the {agent_spec['spatial_relation'].name.replace('_', ' ').lower()} of the ego's lane and "
        elif agent_spec['spatial_relation'] in [SpatialRelation.OPPOSITE_INTERSECTION, SpatialRelation.LATERAL_INTERSECTION]:
            code += f"at the {agent_spec['spatial_relation'].name.replace('_', ' ').lower()} of the intersection and "
        elif agent_spec['spatial_relation'] == PedestrianSpatialRelation.SIDEWALK:
            code += f"on the sidewalk next to the ego vehicle and "
            
        if agent_spec['maneuver'] == VehicleManeuver.STRAIGHT:
            code += f"goes straight in the intersection. "
        elif agent_spec['maneuver'] in [VehicleManeuver.LEFT_TURN, VehicleManeuver.RIGHT_TURN]:
            code += f"makes a {agent_spec['maneuver'].name.replace('_', ' ').lower()} in the intersection. "
        elif agent_spec['maneuver'] == VehicleManeuver.LANE_FOLLOWING:
            code += f"follows its lane. "
        elif agent_spec['maneuver'] == VehicleManeuver.LANE_CHANGE:
            code += f"changes lane when possible. "
        elif agent_spec['maneuver'] == PedestrianManeuver.CROSS_STREET:
            code += f"crosses the street. "
        elif agent_spec['maneuver'] == PedestrianManeuver.WALK_ALONG_SIDEWALK:
            code += f"walks along the sidewalk. "
    code += f'\n'
    code += f'GENERATED BY: auto_scenario_generator.py\n'
    code += f'"""\n'
    code += f"\n"
    return code

def _model_generator(spec):
    code = ""
    code += f"#################################\n"
    code += f"# MAP AND MODEL                 #\n"
    code += f"#################################\n"
    code += f"\n"
    code += f"param map = localPath('{spec['map']}')\n"
    code += f"model scenic.domains.driving.model\n"
    code += f"param POLICY = 'built_in'\n"
    code += f"\n"
    return code

def _constant_generator(spec):
    code = ""
    code += f"#################################\n"
    code += f"# PARAMETERS AND CONSTANTS      #\n"
    code += f"#################################\n"
    code += f"\n"
    for param, (low, high) in parameter_tracker.items():
        code += f"param {param} = VerifaiRange({low}, {high})\n"
    code += f"\n"
    code += f"MODEL = 'vehicle.lincoln.mkz_2017'\n"
    for const, value in constant_tracker.items():
        code += f"{const} = {value}\n"
    code += f"\n"
    return code

def _spatial_generator(spec):
    code = ""
    code += f"#################################\n"
    code += f"# SPATIAL RELATIONS             #\n"
    code += f"#################################\n"
    code += f"\n"
    
    # Ego spatial relations
    assert 'ego' in spec, "Ego agent must be specified in the scenario spec."
    ego = spec['ego']
    if ego['maneuver'] in [VehicleManeuver.STRAIGHT, VehicleManeuver.LEFT_TURN, VehicleManeuver.RIGHT_TURN]:
        # intersection scenario
        code += f"intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))\n"
        code += f"\n"
        code += f"egoInitLane = Uniform(*intersection.incomingLanes)\n"
        code += f"egoSpawnPt = new OrientedPoint in egoInitLane.centerline\n"
        constant_tracker["EGO_INIT_DIST"] = ConstantToRange['INTER_DIST']
        requirement_tracker.append(f"EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]")
    elif ego['maneuver'] in [VehicleManeuver.LANE_FOLLOWING, VehicleManeuver.LANE_CHANGE]:
        code += f"intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))\n"
        code += f"\n"
        code += f"egoInitLane = Uniform(*intersection.incomingLanes)\n"
        code += f"egoSpawnPt = new OrientedPoint in egoInitLane.centerline\n"
    code += f"\n"
    
    # Other agents spatial relations
    for agent_name, agent_spec in spec.get('agents', {}).items():
        if agent_spec['spatial_relation'] == SpatialRelation.AHEAD_OF:
            code += f"{agent_name}SpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.{agent_name.upper()}_DIST\n"
            code += f"{agent_name}InitLane = network.laneAt({agent_name}SpawnPt)\n"
            parameter_tracker[f"{agent_name.upper()}_DIST"] = ParamToRange['INIT_DIST']
        elif agent_spec['spatial_relation'] == SpatialRelation.BEHIND:
            code += f"{agent_name}SpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for -globalParameters.{agent_name.upper()}_DIST\n"
            code += f"{agent_name}InitLane = network.laneAt({agent_name}SpawnPt)\n"
            parameter_tracker[f"{agent_name.upper()}_DIST"] = ParamToRange['INIT_DIST']
        elif agent_spec['spatial_relation'] == SpatialRelation.FASTER_LANE:
            code += f"{agent_name}InitLane = network.laneSectionAt(egoSpawnPt).fasterLane.lane\n"
            code += f"{agent_name}SpawnPt = new OrientedPoint in {agent_name}InitLane.centerline\n"
            requirement_tracker.append(f"network.laneSectionAt(egoSpawnPt).fasterLane.lane is not None")
            constant_tracker[f"{agent_name.upper()}_DIST"] = ConstantToRange['INIT_DIST']
            requirement_tracker.append(f"(distance from {agent_name} to ego) <= {agent_name.upper()}_DIST")
        elif agent_spec['spatial_relation'] == SpatialRelation.SLOWER_LANE:
            code += f"{agent_name}InitLane = network.laneSectionAt(egoSpawnPt).slowerLane.lane\n"
            code += f"{agent_name}SpawnPt = new OrientedPoint in {agent_name}InitLane.centerline\n"
            requirement_tracker.append(f"network.laneSectionAt(egoSpawnPt).slowerLane.lane is not None")
            constant_tracker[f"{agent_name.upper()}_DIST"] = ConstantToRange['INIT_DIST']
            requirement_tracker.append(f"(distance from ego to {agent_name}) <= {agent_name.upper()}_DIST")
        #elif agent_spec['spatial_relation'] == SpatialRelation.OPPOSITE_LANE:
        #    code += f"{agent_name}InitLane = Uniform(*network.laneGroupAt(egoSpawnPt).opposite.lanes)\n"
        #    code += f"{agent_name}SpawnPt = new OrientedPoint in {agent_name}InitLane.centerline\n"
        #    requirement_tracker.append(f"network.laneGroupAt(egoSpawnPt).opposite is not None")
        #    constant_tracker[f"{agent_name.upper()}_DIST"] = ConstantToRange['INIT_DIST']
        #    requirement_tracker.append(f"(distance from ego to {agent_name}) <= 2*{agent_name.upper()}_DIST")
        #    requirement_tracker.append(f"(apparent heading of {agent_name}) > 1.57") # 90 degrees --> in front of ego
        elif agent_spec['spatial_relation'] == SpatialRelation.OPPOSITE_INTERSECTION:
            #assert ego['maneuver'] in [VehicleManeuver.STRAIGHT, VehicleManeuver.LEFT_TURN, VehicleManeuver.RIGHT_TURN], "Ego must be performing intersection maneuver for OPPOSITE_INTERSECTION spatial relation."
            code += f"{agent_name}InitLane = Uniform(*filter(lambda m:\n"
            code += f"    m.type is ManeuverType.STRAIGHT,\n"
            code += f"    Uniform(*filter(lambda m: \n"
            code += f"        m.type is ManeuverType.STRAIGHT, \n"
            code += f"        egoInitLane.maneuvers)\n"
            code += f"    ).reverseManeuvers)\n"
            code += f").startLane\n"
            code += f"{agent_name}SpawnPt = new OrientedPoint in {agent_name}InitLane.centerline\n"
            constant_tracker[f"{agent_name.upper()}_INIT_DIST"] = ConstantToRange['INTER_DIST']
            requirement_tracker.append(f"{agent_name.upper()}_INIT_DIST[0] <= (distance from {agent_name} to intersection) <= {agent_name.upper()}_INIT_DIST[1]")
        elif agent_spec['spatial_relation'] == SpatialRelation.LATERAL_INTERSECTION:
            #assert ego['maneuver'] in [VehicleManeuver.STRAIGHT, VehicleManeuver.LEFT_TURN, VehicleManeuver.RIGHT_TURN], "Ego must be performing intersection maneuver for LATERAL_INTERSECTION spatial relation."
            code += f"{agent_name}InitLane = Uniform(*filter(lambda m:\n"
            code += f"    m.type is ManeuverType.STRAIGHT,\n"
            code += f"    Uniform(*filter(lambda m: \n"
            code += f"        m.type is ManeuverType.STRAIGHT, \n"
            code += f"        egoInitLane.maneuvers)\n"
            code += f"    ).conflictingManeuvers)\n"
            code += f").startLane\n"
            code += f"{agent_name}SpawnPt = new OrientedPoint in {agent_name}InitLane.centerline\n"
            constant_tracker[f"{agent_name.upper()}_INIT_DIST"] = ConstantToRange['INTER_DIST']
            requirement_tracker.append(f"{agent_name.upper()}_INIT_DIST[0] <= (distance from {agent_name} to intersection) <= {agent_name.upper()}_INIT_DIST[1]")
        elif agent_spec['spatial_relation'] == PedestrianSpatialRelation.SIDEWALK:
            code += f"{agent_name}SpawnPt = new OrientedPoint at egoSpawnPt offset by ({agent_name.upper()}_LATERAL_OFFSET, globalParameters.{agent_name.upper()}_LONGITUDINAL_OFFSET, 0)\n"
            if agent_spec['maneuver'] == PedestrianManeuver.CROSS_STREET:
                code += f"{agent_name}EndPt = new OrientedPoint at egoSpawnPt offset by (-{agent_name.upper()}_LATERAL_OFFSET, globalParameters.{agent_name.upper()}_LONGITUDINAL_OFFSET, 0)\n"
            elif agent_spec['maneuver'] == PedestrianManeuver.WALK_ALONG_SIDEWALK:
                code += f"{agent_name}EndPt = new OrientedPoint following roadDirection from {agent_name}SpawnPt for {agent_name.upper()}_LONGITUDINAL_OFFSET\n"
                constant_tracker[f"{agent_name.upper()}_LONGITUDINAL_OFFSET"] = ConstantToRange['PED_LONGITUDINAL_OFFSET']
            constant_tracker[f"{agent_name.upper()}_LATERAL_OFFSET"] = ConstantToRange['PED_LATERAL_OFFSET']
            parameter_tracker[f"{agent_name.upper()}_LONGITUDINAL_OFFSET"] = ParamToRange['PED_LONGITUDINAL_OFFSET']
        else:
            raise ValueError(f"Unsupported spatial relation: {agent_spec['spatial_relation']}")
        code += f"\n"

    return code

def _behavior_generator(spec):
    dist_func_defined = False
    
    code = ""
    code += f"#################################\n"
    code += f"# AGENT BEHAVIORS               #\n"
    code += f"#################################\n"
    code += f"\n"
    
    # Ego behavior
    assert 'ego' in spec, "Ego agent must be specified in the scenario spec."
    ego = spec['ego']
    if ego['maneuver'] in [VehicleManeuver.STRAIGHT]:
        code += f"egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.{ego['maneuver'].name}, egoInitLane.maneuvers))\n"
        code += f"egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]\n"
        code += f"behavior EgoBehavior(trajectory):\n"
        code += f"    try:\n"
        code += f"        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)\n"
        #code += f"    interrupt when withinDistanceToAnyObjs(self, globalParameters.EGO_SAFETY_DIST):\n"
        code += f"    interrupt when withinDistanceToObjsInLane(self, globalParameters.EGO_SAFETY_DIST):\n"
        code += f"        take SetBrakeAction(globalParameters.EGO_BRAKE)\n"
        
        parameter_tracker["EGO_SPEED"] = ParamToRange['SPEED']
        parameter_tracker["EGO_BRAKE"] = ParamToRange['BRAKE']
        parameter_tracker["EGO_SAFETY_DIST"] = ParamToRange['SAFETY_DIST']
    elif ego['maneuver'] in [VehicleManeuver.LEFT_TURN, VehicleManeuver.RIGHT_TURN]:
        code += f"egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.{ego['maneuver'].name}, egoInitLane.maneuvers))\n"
        code += f"egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]\n"
        code += f"behavior EgoBehavior(trajectory):\n"
        code += f"    try:\n"
        code += f"        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)\n"
        code += f"    interrupt when withinDistanceToAnyObjs(self, globalParameters.EGO_SAFETY_DIST):\n"
        #code += f"    interrupt when withinDistanceToObjsInLane(self, globalParameters.EGO_SAFETY_DIST):\n"
        code += f"        take SetBrakeAction(globalParameters.EGO_BRAKE)\n"
        
        parameter_tracker["EGO_SPEED"] = ParamToRange['SPEED']
        parameter_tracker["EGO_BRAKE"] = ParamToRange['BRAKE']
        parameter_tracker["EGO_SAFETY_DIST"] = ParamToRange['SAFETY_DIST']
    elif ego['maneuver'] in [VehicleManeuver.LANE_FOLLOWING]:
        code += f"def withinDistanceToObjsInLaneNoInter(vehicle, thresholdDistance):\n"
        code += f"    objects = simulation().objects\n"
        code += f"    for obj in objects:\n"
        code += f"        if not (vehicle can see obj):\n"
        code += f"            continue\n"
        code += f"        if (distance from vehicle.position to obj.position) < 0.1:\n"
        code += f"            # this means obj==vehicle\n"
        code += f"            continue\n"
        code += f"        if network.laneAt(vehicle) != network.laneAt(obj):    # different lanes\n"
        code += f"            continue\n"
        code += f"        if (distance from vehicle to obj) < thresholdDistance:\n"
        code += f"            return True\n"
        code += f"    return False\n"
        code += f"\n"
        code += f"behavior EgoBehavior():\n"
        code += f"    try:\n"
        code += f"        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)\n"
        #code += f"    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):\n"
        code += f"    interrupt when withinDistanceToObjsInLaneNoInter(self, globalParameters.EGO_SAFETY_DIST):\n"
        code += f"        take SetBrakeAction(globalParameters.EGO_BRAKE)\n"
        
        parameter_tracker["EGO_SPEED"] = ParamToRange['SPEED']
        parameter_tracker["EGO_BRAKE"] = ParamToRange['BRAKE']
        parameter_tracker["EGO_SAFETY_DIST"] = ParamToRange['SAFETY_DIST']
        dist_func_defined = True
    elif ego['maneuver'] in [VehicleManeuver.LANE_CHANGE]:
        code += f"def withinDistanceToObjsInLaneNoInter(vehicle, thresholdDistance):\n"
        code += f"    objects = simulation().objects\n"
        code += f"    for obj in objects:\n"
        code += f"        if not (vehicle can see obj):\n"
        code += f"            continue\n"
        code += f"        if (distance from vehicle.position to obj.position) < 0.1:\n"
        code += f"            # this means obj==vehicle\n"
        code += f"            continue\n"
        code += f"        if network.laneAt(vehicle) != network.laneAt(obj):    # different lanes\n"
        code += f"            continue\n"
        code += f"        if (distance from vehicle to obj) < thresholdDistance:\n"
        code += f"            return True\n"
        code += f"    return False\n"
        code += f"\n"
        code += f"behavior EgoBehavior():\n"
        code += f"    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) for 1 seconds\n"
        code += f"    try:\n"
        code += f"        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)\n"
        code += f"    interrupt when (ego.laneSection._fasterLane is not None) and "
        for agent_name in spec.get('agents', {}).keys():
            code += f"(distance to {agent_name} >= globalParameters.EGO_BYPASS_DIST) and "
        code += f"(not self.switched):\n"
        code += f"        self.switched = True\n"
        code += f"        do LaneChangeBehavior(target_speed=globalParameters.EGO_SPEED, laneSectionToSwitch=ego.laneSection._fasterLane)\n"
        code += f"        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)\n"
        code += f"    interrupt when (ego.laneSection._slowerLane is not None) and "
        for agent_name in spec.get('agents', {}).keys():
            code += f"(distance to {agent_name} >= globalParameters.EGO_BYPASS_DIST) and "
        code += f"(not self.switched):\n"
        code += f"        self.switched = True\n"
        code += f"        do LaneChangeBehavior(target_speed=globalParameters.EGO_SPEED, laneSectionToSwitch=ego.laneSection._slowerLane)\n"
        code += f"        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)\n"
        code += f"    interrupt when withinDistanceToObjsInLaneNoInter(self, globalParameters.EGO_SAFETY_DIST):\n"
        code += f"        take SetBrakeAction(globalParameters.EGO_BRAKE)\n"
        
        parameter_tracker["EGO_SPEED"] = ParamToRange['SPEED']
        parameter_tracker["EGO_BRAKE"] = ParamToRange['BRAKE']
        parameter_tracker["EGO_SAFETY_DIST"] = ParamToRange['SAFETY_DIST']
        parameter_tracker["EGO_BYPASS_DIST"] = ParamToRange['BYPASS_DIST']
        dist_func_defined = True
    code += f"\n"
    
    # Other agents behavior
    for agent_name, agent_spec in spec.get('agents', {}).items():
        if agent_spec['maneuver'] in [VehicleManeuver.STRAIGHT, VehicleManeuver.LEFT_TURN, VehicleManeuver.RIGHT_TURN]:
            requirement_tracker.append(f"{agent_name}InitLane in intersection.incomingLanes")
            code += f"{agent_name}Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.{agent_spec['maneuver'].name}, {agent_name}InitLane.maneuvers))\n"
            code += f"{agent_name}Trajectory = [{agent_name}InitLane, {agent_name}Maneuver.connectingLane, {agent_name}Maneuver.endLane]\n"
            code += f"behavior {agent_name.capitalize()}Behavior(trajectory):\n"
            if agent_spec.get('strategy') == 'conservative':
                code += f"    try:\n"
                code += f"        do FollowTrajectoryBehavior(target_speed=globalParameters.{agent_name.upper()}_SPEED, trajectory=trajectory)\n"
                code += f"    interrupt when withinDistanceToAnyObjs(self, globalParameters.{agent_name.upper()}_SAFETY_DIST):\n"
                #code += f"    interrupt when withinDistanceToObjsInLane(self, globalParameters.{agent_name.upper()}_SAFETY_DIST):\n"
                code += f"        take SetBrakeAction(globalParameters.{agent_name.upper()}_BRAKE)\n"
                parameter_tracker[f"{agent_name.upper()}_SPEED"] = ParamToRange['SPEED']
                parameter_tracker[f"{agent_name.upper()}_BRAKE"] = ParamToRange['BRAKE']
                parameter_tracker[f"{agent_name.upper()}_SAFETY_DIST"] = ParamToRange['SAFETY_DIST']
            else:
                code += f"    do FollowTrajectoryBehavior(target_speed=globalParameters.{agent_name.upper()}_SPEED, trajectory=trajectory)\n"
                parameter_tracker[f"{agent_name.upper()}_SPEED"] = ParamToRange['SPEED']
        elif agent_spec['maneuver'] in [VehicleManeuver.LANE_FOLLOWING]:
            if not dist_func_defined:
                code += f"def withinDistanceToObjsInLaneNoInter(vehicle, thresholdDistance):\n"
                code += f"    objects = simulation().objects\n"
                code += f"    for obj in objects:\n"
                code += f"        if not (vehicle can see obj):\n"
                code += f"            continue\n"
                code += f"        if (distance from vehicle.position to obj.position) < 0.1:\n"
                code += f"            # this means obj==vehicle\n"
                code += f"            continue\n"
                code += f"        if network.laneAt(vehicle) != network.laneAt(obj):    # different lanes\n"
                code += f"            continue\n"
                code += f"        if (distance from vehicle to obj) < thresholdDistance:\n"
                code += f"            return True\n"
                code += f"    return False\n"
                code += f"\n"
                dist_func_defined = True
            code += f"behavior {agent_name.capitalize()}Behavior():\n"
            if agent_spec.get('strategy') == 'conservative':
                code += f"    try:\n"
                code += f"        do FollowLaneBehavior(target_speed=globalParameters.{agent_name.upper()}_SPEED)\n"
                #code += f"    interrupt when withinDistanceToAnyObjs(self, globalParameters.{agent_name.upper()}_SAFETY_DIST):\n"
                code += f"    interrupt when withinDistanceToObjsInLaneNoInter(self, globalParameters.{agent_name.upper()}_SAFETY_DIST):\n"
                code += f"        take SetBrakeAction(globalParameters.{agent_name.upper()}_BRAKE)\n"
                parameter_tracker[f"{agent_name.upper()}_SPEED"] = ParamToRange['SPEED']
                parameter_tracker[f"{agent_name.upper()}_BRAKE"] = ParamToRange['BRAKE']
                parameter_tracker[f"{agent_name.upper()}_SAFETY_DIST"] = ParamToRange['SAFETY_DIST']
            else:
                code += f"    do FollowLaneBehavior(target_speed=globalParameters.{agent_name.upper()}_SPEED)\n"
                parameter_tracker[f"{agent_name.upper()}_SPEED"] = ParamToRange['SPEED']
        elif agent_spec['maneuver'] in [VehicleManeuver.LANE_CHANGE]:
            if not dist_func_defined:
                code += f"def withinDistanceToObjsInLaneNoInter(vehicle, thresholdDistance):\n"
                code += f"    objects = simulation().objects\n"
                code += f"    for obj in objects:\n"
                code += f"        if not (vehicle can see obj):\n"
                code += f"            continue\n"
                code += f"        if (distance from vehicle.position to obj.position) < 0.1:\n"
                code += f"            # this means obj==vehicle\n"
                code += f"            continue\n"
                code += f"        if network.laneAt(vehicle) != network.laneAt(obj):    # different lanes\n"
                code += f"            continue\n"
                code += f"        if (distance from vehicle to obj) < thresholdDistance:\n"
                code += f"            return True\n"
                code += f"    return False\n"
                code += f"\n"
                dist_func_defined = True
            code += f"behavior {agent_name.capitalize()}Behavior():\n"
            code += f"    do FollowLaneBehavior(target_speed=globalParameters.{agent_name.upper()}_SPEED) for 1 seconds\n"
            code += f"    try:\n"
            code += f"        do FollowLaneBehavior(target_speed=globalParameters.{agent_name.upper()}_SPEED)\n"
            code += f"    interrupt when (self.laneSection._fasterLane is not None) and "
            for other_agent_name in spec.get('agents', {}).keys():
                if other_agent_name != agent_name:
                    code += f"(distance from {agent_name} to {other_agent_name} >= globalParameters.{agent_name.upper()}_BYPASS_DIST) and "
            code += f"(distance from {agent_name} to ego >= globalParameters.{agent_name.upper()}_BYPASS_DIST) and "
            code += f"(not self.switched):\n"
            code += f"        self.switched = True\n"
            code += f"        do LaneChangeBehavior(target_speed=globalParameters.{agent_name.upper()}_SPEED, laneSectionToSwitch=self.laneSection._fasterLane)\n"
            code += f"        do FollowLaneBehavior(target_speed=globalParameters.{agent_name.upper()}_SPEED)\n"
            code += f"    interrupt when (self.laneSection._slowerLane is not None) and "
            for other_agent_name in spec.get('agents', {}).keys():
                if other_agent_name != agent_name:
                    code += f"(distance from {agent_name} to {other_agent_name} >= globalParameters.{agent_name.upper()}_BYPASS_DIST) and "
            code += f"(distance from {agent_name} to ego >= globalParameters.{agent_name.upper()}_BYPASS_DIST) and "
            code += f"(not self.switched):\n"
            code += f"        self.switched = True\n"
            code += f"        do LaneChangeBehavior(target_speed=globalParameters.{agent_name.upper()}_SPEED, laneSectionToSwitch=self.laneSection._slowerLane)\n"
            code += f"        do FollowLaneBehavior(target_speed=globalParameters.{agent_name.upper()}_SPEED)\n"
            if agent_spec.get('strategy') == 'conservative':
                code += f"    interrupt when withinDistanceToObjsInLaneNoInter(self, globalParameters.{agent_name.upper()}_SAFETY_DIST):\n"
                code += f"        take SetBrakeAction(globalParameters.{agent_name.upper()}_BRAKE)\n"
                parameter_tracker[f"{agent_name.upper()}_SPEED"] = ParamToRange['SPEED']
                parameter_tracker[f"{agent_name.upper()}_BRAKE"] = ParamToRange['BRAKE']
                parameter_tracker[f"{agent_name.upper()}_SAFETY_DIST"] = ParamToRange['SAFETY_DIST']
                parameter_tracker[f"{agent_name.upper()}_BYPASS_DIST"] = ParamToRange['BYPASS_DIST']
            else:
                parameter_tracker[f"{agent_name.upper()}_SPEED"] = ParamToRange['SPEED']
                parameter_tracker[f"{agent_name.upper()}_BYPASS_DIST"] = ParamToRange['BYPASS_DIST']
        elif agent_spec['maneuver'] in [PedestrianManeuver.CROSS_STREET, PedestrianManeuver.WALK_ALONG_SIDEWALK]:
            code += f"behavior {agent_name.capitalize()}Behavior():\n"
            code += f"    take SetWalkingSpeedAction(speed=globalParameters.{agent_name.upper()}_SPEED)\n"
            parameter_tracker[f"{agent_name.upper()}_SPEED"] = ParamToRange['PED_SPEED']
        else:
            raise ValueError(f"Unsupported maneuver: {agent_spec['maneuver']}")
        code += f"\n"
    
    return code

def _specification_generator(spec):
    lane_changed_car_defined = False
    
    code = ""
    code += f"#################################\n"
    code += f"# SPECIFICATIONS                #\n"
    code += f"#################################\n"
    code += f"\n"
    
    # Ego specifications
    ego = spec['ego']
    if ego['maneuver'] in [VehicleManeuver.LANE_CHANGE]:
        code += f"class LaneChangeCar(Car):\n"
        code += f"    switched: False\n"
        code += f"\n"
        code += f"if globalParameters.POLICY == 'metadrive_ppo' or globalParameters.POLICY == 'ppo_with_built_in':\n"
        code += f"    from metadrive_expert import MetaDrivePPOPolicyCar, MetaDrivePPOFollowLaneBehavior, MetaDrivePPOUpdateState\n"
        code += f"    behavior EgoPPOBehavior():\n"
        code += f"        do MetaDrivePPOFollowLaneBehavior() for 1 seconds\n"
        code += f"        try:\n"
        code += f"            do MetaDrivePPOFollowLaneBehavior()\n"
        code += f"        interrupt when (ego.laneSection._fasterLane is not None) and "
        for agent_name in spec.get('agents', {}).keys():
            code += f"(distance to {agent_name} >= globalParameters.EGO_BYPASS_DIST) and "
        code += f"(not self.switched):\n"
        code += f"            self.switched = True\n"
        code += f"            do LaneChangeBehavior(target_speed=globalParameters.EGO_SPEED, laneSectionToSwitch=ego.laneSection._fasterLane)\n"
        code += f"            do MetaDrivePPOFollowLaneBehavior()\n"
        code += f"        interrupt when (ego.laneSection._slowerLane is not None) and "
        for agent_name in spec.get('agents', {}).keys():
            code += f"(distance to {agent_name} >= globalParameters.EGO_BYPASS_DIST) and "
        code += f"(not self.switched):\n"
        code += f"            self.switched = True\n"
        code += f"            do LaneChangeBehavior(target_speed=globalParameters.EGO_SPEED, laneSectionToSwitch=ego.laneSection._slowerLane)\n"
        code += f"            do MetaDrivePPOFollowLaneBehavior()\n"
        code += f"        interrupt when withinDistanceToObjsInLaneNoInter(self, globalParameters.EGO_SAFETY_DIST):\n"
        code += f"            take SetBrakeAction(globalParameters.EGO_BRAKE)\n"
        code += f"    ego = new MetaDrivePPOPolicyCar at egoSpawnPt,\n"
        code += f"          with blueprint MODEL,\n"
        code += f"          with behavior EgoPPOBehavior()\n"
        code += f"    require monitor MetaDrivePPOUpdateState()\n"
        code += f"else:\n"
        code += f"    ego = new LaneChangeCar at egoSpawnPt,\n"
        lane_changed_car_defined = True
    elif ego['maneuver'] in [VehicleManeuver.LANE_FOLLOWING]:
        code += f"if globalParameters.POLICY == 'metadrive_ppo' or globalParameters.POLICY == 'ppo_with_built_in':\n"
        code += f"    from metadrive_expert import MetaDrivePPOPolicyCar, MetaDrivePPOFollowLaneBehavior, MetaDrivePPOUpdateState\n"
        code += f"    ego = new MetaDrivePPOPolicyCar at egoSpawnPt,\n"
        code += f"          with blueprint MODEL,\n"
        code += f"          with behavior MetaDrivePPOFollowLaneBehavior()\n"
        code += f"    require monitor MetaDrivePPOUpdateState()\n"
        code += f"else:\n"
        code += f"    ego = new {spec['ego']['type'].name.capitalize()} at egoSpawnPt,\n"
    else:
        code += f"if globalParameters.POLICY == 'metadrive_ppo' or globalParameters.POLICY == 'ppo_with_built_in':\n"
        code += f"    from metadrive_expert import MetaDrivePPOPolicyCar, MetaDrivePPOPolicyBehavior, MetaDrivePPOUpdateState\n"
        code += f"    ego = new MetaDrivePPOPolicyCar at egoSpawnPt,\n"
        code += f"          with blueprint MODEL,\n"
        code += f"          with behavior MetaDrivePPOPolicyBehavior(egoTrajectory)\n"
        code += f"    require monitor MetaDrivePPOUpdateState()\n"
        code += f"else:\n"
        code += f"    ego = new {spec['ego']['type'].name.capitalize()} at egoSpawnPt,\n"
    code += f"          with blueprint MODEL,\n"
    if spec['ego']['maneuver'] in [VehicleManeuver.STRAIGHT, VehicleManeuver.LEFT_TURN, VehicleManeuver.RIGHT_TURN]:
        code += f"          with behavior EgoBehavior(egoTrajectory)\n"
    elif spec['ego']['maneuver'] in [VehicleManeuver.LANE_FOLLOWING, VehicleManeuver.LANE_CHANGE]:
        code += f"          with behavior EgoBehavior()\n"
    code += f"\n"
    
    # Other agents specifications
    for agent_name, agent_spec in spec.get('agents', {}).items():
        # Pedestrian agents
        if agent_spec['type'] == AgentType.PEDESTRIAN:
            code += f"{agent_name} = new {agent_spec['type'].name.capitalize()} at {agent_name}SpawnPt, facing toward {agent_name}EndPt,\n"
            code += f"      with behavior {agent_name.capitalize()}Behavior()\n"
            code += f"\n"
            continue
        
        # Vehicle agents
        if agent_spec['maneuver'] in [VehicleManeuver.LANE_CHANGE]:
            if not lane_changed_car_defined:
                code += f"class LaneChangeCar(Car):\n"
                code += f"    switched: False\n"
                code += f"\n"
            code += f"{agent_name} = new LaneChangeCar at {agent_name}SpawnPt,\n"
            lane_changed_car_defined = True
        else:
            code += f"{agent_name} = new {agent_spec['type'].name.capitalize()} at {agent_name}SpawnPt,\n"
        code += f"      with blueprint MODEL,\n"
        if agent_spec['maneuver'] in [VehicleManeuver.STRAIGHT, VehicleManeuver.LEFT_TURN, VehicleManeuver.RIGHT_TURN]:
            code += f"      with behavior {agent_name.capitalize()}Behavior({agent_name}Trajectory)\n"
        elif agent_spec['maneuver'] in [VehicleManeuver.LANE_FOLLOWING, VehicleManeuver.LANE_CHANGE]:
            code += f"      with behavior {agent_name.capitalize()}Behavior()\n"
        
        code += f"\n"
        
    # Requirements
    if requirement_tracker:
        for req in requirement_tracker:
            code += f"require {req}\n"
            
    # Termination
    # TODO
    
    code += f"\n"
    return code

def _recording_generator(spec):
    code = ""
    code += f"#################################\n"
    code += f"# RECORDING                     #\n"
    code += f"#################################\n"
    code += f"\n"
    code += "from rulebook_benchmark import bench\n"
    code += "require monitor bench.bench()\n"
    code += f"\n"
    if spec['ego']['maneuver'] in [VehicleManeuver.STRAIGHT, VehicleManeuver.LEFT_TURN, VehicleManeuver.RIGHT_TURN]:
        code += f"record ego in egoManeuver.endLane as egoReachedGoal\n"
    elif spec['ego']['maneuver'] in [VehicleManeuver.LANE_FOLLOWING]:
        code += f"record True as egoReachedGoal\n"
    elif spec['ego']['maneuver'] in [VehicleManeuver.LANE_CHANGE]:
        code += f"record ego.switched as egoReachedGoal\n"
    code += f"record ego._boundingPolygon as egoPoly\n"
    code += f"record ego.lane.polygon as egoLanePoly\n"
    for agent_name in spec.get('agents', {}).keys():
        code += f"record {agent_name}._boundingPolygon as {agent_name}Poly\n"
        if spec['agents'][agent_name]['type'] == AgentType.CAR:
            code += f"record {agent_name}.lane.polygon as {agent_name}LanePoly\n"
    return code

def generate_scenario_from_file(file):
    """Given a JSONL file containing multiple scenario specs, generate scenarios for each spec. Each line in the file should be a valid JSON object representing a scenario spec.

    Args:
        file (str): The path to the JSONL file.
    """
    with open(file, 'r') as f:
        for line_num, line in enumerate(f, start=1):
            if not line.strip():
                continue  # skip empty lines

            try:
                raw_spec = json.loads(line)
            except json.JSONDecodeError as e:
                print(f"[Line {line_num}] JSON decode error: {e}")
                continue

            try:
                # --- Convert string enums to actual Enum objects ---
                spec = {
                    'scenario': raw_spec['scenario'],
                    'map': raw_spec['map'],
                    'ego': {
                        'type': AgentType[raw_spec['ego']['type']],
                        'maneuver': VehicleManeuver[raw_spec['ego']['maneuver']],
                    },
                    'agents': {}
                }

                for aid, agent in raw_spec['agents'].items():
                    converted_agent = {
                        'type': AgentType[agent['type']],
                        'maneuver': (
                            VehicleManeuver[agent['maneuver']]
                            if agent['type'] == 'CAR'
                            else PedestrianManeuver[agent['maneuver']]
                        ),
                    }
                    if 'strategy' in agent:
                        converted_agent['strategy'] = agent['strategy']
                    if 'spatial_relation' in agent:
                        # Decide which enum type to use
                        if agent['type'] == 'CAR':
                            converted_agent['spatial_relation'] = SpatialRelation[agent['spatial_relation']]
                        elif agent['type'] == 'PEDESTRIAN':
                            converted_agent['spatial_relation'] = PedestrianSpatialRelation[agent['spatial_relation']]
                    spec['agents'][aid] = converted_agent

                # --- Check and generate scenario ---
                scenario_spec_checker(spec)
                scenario_generator(spec)
                print(f"[Scenario {line_num}] Scenario generated successfully.")

            except ValueError as e:
                print(f"[Scenario {line_num}] Spec invalid: {e}")
            except KeyError as e:
                print(f"[Scenario {line_num}] Missing or unknown enum key: {e}")
            except Exception as e:
                print(f"[Scenario {line_num}] Unexpected error: {e}")

def generate_random_scenario_specs(
    jsonl_filename: str,
    num_vehicle_agents: int,
    num_ped_agents: int,
    num_scenarios: int,
    max_retries: int = 50
):
    """
    Generate a JSONL file containing valid random scenario specs.
    
    Args:
        jsonl_filename (str): The output JSONL file name.
        num_vehicle_agents (int): Number of vehicle agents in each scenario (excluding ego).
        num_ped_agents (int): Number of pedestrian agents in each scenario.
        num_scenarios (int): Number of valid scenarios to generate.
        max_retries (int): Maximum number of attempts to generate valid scenarios.
    """
    random.seed(42)  # for reproducibility
    
    valid_scenarios = []
    unique_specs = set()  # store JSON string representations to detect duplicates
    retries = 0
    
    def enum_to_str(obj):
        if isinstance(obj, Enum):
            return obj.name
        raise TypeError(f"Type {type(obj)} not serializable")

    while len(valid_scenarios) < num_scenarios and retries < max_retries:
        retries += 1
        scenario_name = f"basic_gen/basic{num_vehicle_agents}{num_ped_agents}_{len(valid_scenarios)+1}.scenic"
        spec = {
            "scenario": scenario_name,
            "map": "../../maps/Town05.xodr",
            "ego": {
                "type": AgentType.CAR,
                "maneuver": random.choice(list(VehicleManeuver)),
            },
            "agents": {}
        }

        # Add vehicle agents
        for i in range(num_vehicle_agents):
            agent_name = f"car{i+1}"
            spec["agents"][agent_name] = {
                "type": AgentType.CAR,
                "maneuver": random.choice(list(VehicleManeuver)),
                "strategy": random.choice(["aggressive", "conservative"]),
                "spatial_relation": random.choice(list(SpatialRelation)),
            }

        # Add pedestrian agents
        for i in range(num_ped_agents):
            agent_name = f"ped{i+1}"
            spec["agents"][agent_name] = {
                "type": AgentType.PEDESTRIAN,
                "maneuver": random.choice(list(PedestrianManeuver)),
                "spatial_relation": random.choice(list(PedestrianSpatialRelation)),
            }
            
        # Create canonical form (convert Enums to strings and sort keys)
        spec_json = json.dumps(spec, default=enum_to_str, sort_keys=True)

        # Check if duplicate
        if spec_json in unique_specs:
            print(f"Duplicate spec generated (retry {retries}), skipping.")
            continue

        # Check validity
        try:
            scenario_spec_checker(spec)
            valid_scenarios.append(spec)
            unique_specs.add(spec_json)
            print(f"Valid scenario generated ({len(valid_scenarios)}/{num_scenarios})")
            retries = 0  # reset retries after a successful generation
        except ValueError as e:
            print(f"Invalid spec (retry {retries}): {e}")
            continue

    if len(valid_scenarios) < num_scenarios:
        print(f"[Warning!] Only {len(valid_scenarios)} valid scenarios generated after {retries} attempts.")

    # Write valid specs to jsonl file
    with open(jsonl_filename, "w") as f:
        for spec in valid_scenarios:
            f.write(json.dumps(spec, default=enum_to_str) + "\n")
            
if __name__ == "__main__":
    # Example: Generate random scenario specs and save to a JSONL file
    generate_random_scenario_specs(
        jsonl_filename="basic_specs/basic_scenarios_10.jsonl",
        num_vehicle_agents=1,
        num_ped_agents=0,
        num_scenarios=50
    )
    
    # Example: Generate Scenic programs from a JSONL file containing multiple specs
    generate_scenario_from_file('basic_specs/basic_scenarios_10.jsonl')
    
    # Example scenario spec
    #spec = {
    #    'scenario': 'basic_gen/basic_test.scenic',
    #    'map': '../../maps/Town05.xodr',
    #    'ego': {
    #        'type': AgentType.CAR,
    #        'maneuver': VehicleManeuver.RIGHT_TURN,
    #    },
    #    'agents': {
    #        'agent1': {
    #            'type': AgentType.CAR,
    #            'maneuver': VehicleManeuver.LANE_FOLLOWING,
    #            'strategy': 'conservative',
    #            'spatial_relation': SpatialRelation.FASTER_LANE,
    #        },
    #        'agent2': {
    #            'type': AgentType.CAR,
    #            'maneuver': VehicleManeuver.LANE_FOLLOWING,
    #            'strategy': 'aggressive',
    #            'spatial_relation': SpatialRelation.AHEAD_OF,
    #        },
    #        'agent3': {
    #            'type': AgentType.PEDESTRIAN,
    #            'maneuver': PedestrianManeuver.CROSS_STREET,
    #            'spatial_relation': PedestrianSpatialRelation.SIDEWALK,
    #        },
    #    }
    #}
    
    # Example: Check spec and generate scenario
    #scenario_spec_checker(spec)
    #scenario_generator(spec)
    