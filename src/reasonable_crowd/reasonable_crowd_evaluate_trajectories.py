import os
import sys
from reasonable_crowd_evaluate_rules import evaluate_rules

path_to_reasonable_crowd = "../../../Reasonable-Crowd"
u_boundary_path = os.path.join(path_to_reasonable_crowd, "maps/U_boundaries.gpkg")
u_road_path = os.path.join(path_to_reasonable_crowd, "maps/U_road_segments.gpkg")
u_intersection_path = os.path.join(path_to_reasonable_crowd, "maps/U_intersections.gpkg")
u_lane_group_path = os.path.join(path_to_reasonable_crowd, "maps/U_lane_groups_polygons.gpkg")
u_lane_path = os.path.join(path_to_reasonable_crowd, "maps/U_lanes_polygons.gpkg")
s_boundary_path = os.path.join(path_to_reasonable_crowd, "maps/S_boundaries.gpkg")
s_road_path = os.path.join(path_to_reasonable_crowd, "maps/S_road_segments.gpkg")
s_intersection_path = os.path.join(path_to_reasonable_crowd, "maps/S_intersections.gpkg")
s_lane_group_path = os.path.join(path_to_reasonable_crowd, "maps/S_lane_groups_polygons.gpkg")
s_lane_path = os.path.join(path_to_reasonable_crowd, "maps/S_lanes_polygons.gpkg")

input_folder = "../../../Reasonable-Crowd/trajectories"
output_folder = "outputs"
if len(sys.argv) < 2:
    print("Usage: python reasonable_crowd_evaluate_trajectories.py <output_file>")
    sys.exit(1)
output_file = os.path.join(output_folder, sys.argv[1])

results = []
for filename in os.listdir(input_folder):
    traj_path = os.path.join(input_folder, filename)
    print(f"Evaluating {filename}")
    if filename.startswith("U"):
        result = evaluate_rules(traj_path, u_boundary_path, u_road_path, u_intersection_path, u_lane_group_path, u_lane_path)
    else:
        result = evaluate_rules(traj_path, s_boundary_path, s_road_path, s_intersection_path, s_lane_group_path, s_lane_path)
    filename = filename.split(".")[0]
    results.append((filename, result))
results.sort(key=lambda x: x[0])

with open(output_file, 'w') as out_f:
    for filename, result in results:
        result_line = f"{filename} " + " ".join(map(str, result))
        out_f.write(result_line + "\n")
out_f.close()
