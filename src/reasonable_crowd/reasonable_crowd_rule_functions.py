def rule_ped_collision(traj: tuple, start_index, end_index):
    name, results = traj
    return results[name][0]

def rule_vehicle_collision(traj: tuple, start_index, end_index):
    name, results = traj
    return results[name][1]

def rule_drivable_area(traj: tuple, start_index, end_index):
    name, results = traj
    return results[name][2]

def rule_ped_time_to_collision(traj: tuple, start_index, end_index):
    name, results = traj
    return results[name][3]

def rule_vehicle_time_to_collision(traj: tuple, start_index, end_index):
    name, results = traj
    return results[name][4]

def rule_correct_side_of_road(traj: tuple, start_index, end_index):
    name, results = traj
    return results[name][5]

def rule_ped_clearance(traj: tuple, start_index, end_index):
    name, results = traj
    return results[name][6]

def rule_vehicle_clearance(traj: tuple, start_index, end_index):
    name, results = traj
    return results[name][7]

def rule_lane_keeping(traj: tuple, start_index, end_index):
    name, results = traj
    return results[name][8]

def rule_speed_limit(traj: tuple, start_index, end_index):
    name, results = traj
    return results[name][9]

def rule_lane_centering(traj: tuple, start_index, end_index):
    name, results = traj
    return results[name][10]
