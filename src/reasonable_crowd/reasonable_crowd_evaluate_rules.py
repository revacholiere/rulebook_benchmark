import json
import os
import geopandas as gpd
import numpy as np
import shapely
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from math import sqrt

def evaluate_rules(
    traj_path: os.path,
    boundary_path: os.path,
    road_path: os.path,
    intersection_path: os.path,
    lane_group_path: os.path,
    lane_path: os.path,
    to_print=False
) -> None:
    # Parse the trajectory and map files
    with open(traj_path, "r") as j_file:
        traj_states = json.load(j_file)
    boundary_df = gpd.read_file(boundary_path)
    road_df = gpd.read_file(road_path)
    road_polygon = unary_union(road_df.geometry)
    intersection_df = gpd.read_file(intersection_path)
    intersection_polygon = unary_union(intersection_df.geometry)
    lane_group_df = gpd.read_file(lane_group_path)
    lane_group_polygons = [Polygon(geom) for geom in lane_group_df.geometry]
    lane_df = gpd.read_file(lane_path)
    lane_polygons = [Polygon(geom) for geom in lane_df.geometry]
    
    # Number of states
    ego_number_of_states = 0
    vehicle_number_of_states = 0
    ped_number_of_states = 0
    
    # Evaluation results
    ped_collision = 0
    ped_clearance = 0
    vehicle_collision = 0
    vehicle_clearance = 0
    drivable_area = 0
    correct_side_of_road = 0
    lane_keeping = 0
    speed_limit = 0
    lane_centering = 0
    vehicle_time_to_collision = 0
    ped_time_to_collision = 0
    
    # Thresholds
    ped_clearance_threshold = 1  # meters; the larger, the more strict
    vehicle_clearance_threshold = 0.8  # meters; the larger, the more strict
    speed_limit_threshold = 14  # meters per second; the larger, the more lenient
    lane_centering_buffer = 0.3  # meters; the larger, the more strict
    vehicle_time_to_collision_threshold = 0.8  # seconds; the larger, the more strict
    ped_time_to_collision_threshold = 1.0  # seconds; the larger, the more strict
    
    # Set the initial state of the ego vehicle
    curr_ego_state = traj_states[0]
    curr_ego_polygon = Polygon(curr_ego_state["footprint"])
    curr_ego_pos = Point(curr_ego_state["x_meters"], curr_ego_state["y_meters"])
    curr_ego_lane_group = -1
    for idx, lane_group_polygon in enumerate(lane_group_polygons):
        curr_ego_pos = Point(curr_ego_state["x_meters"], curr_ego_state["y_meters"])
        if lane_group_polygon.contains(curr_ego_pos):
            curr_ego_lane_group = idx
            break
    prev_ego_lane_group = -1
    curr_ego_lane = -1
    for idx, lane_polygon in enumerate(lane_polygons):
        curr_ego_pos = Point(curr_ego_state["x_meters"], curr_ego_state["y_meters"])
        if lane_polygon.contains(curr_ego_pos):
            curr_ego_lane = idx
            break
    prev_ego_lane = -1
    
    min_value = 0
    value_1 = 0
    value_2 = 0
    value_3 = 0
    value_4 = 0
    
    for i in range(len(traj_states)):
        curr_state = traj_states[i]
        
        # The current state is the ego vehicle
        if curr_state["type"] == "ego":
            ego_number_of_states += 1
            curr_ego_state = curr_state
            curr_ego_polygon = Polygon(curr_ego_state["footprint"])
            
            # drivable area check
            drivable_area_polygon = road_polygon.union(intersection_polygon)
            if not drivable_area_polygon.contains(curr_ego_polygon):
                drivable_area += 1
                #drivable_area = max(drivable_area, curr_ego_polygon.difference(drivable_area_polygon).area)
                
            # correct side of the road check
            prev_ego_lane_group = curr_ego_lane_group
            for idx, lane_group_polygon in enumerate(lane_group_polygons):
                curr_ego_pos = Point(curr_ego_state["x_meters"], curr_ego_state["y_meters"])
                if lane_group_polygon.contains(curr_ego_pos):
                    curr_ego_lane_group = idx
                    break
                curr_ego_lane_group = -1
            if prev_ego_lane_group != curr_ego_lane_group:
                if curr_ego_lane_group != -1 and prev_ego_lane_group != -1:
                    correct_side_of_road += 1
            
            # lane keeping check
            prev_ego_lane = curr_ego_lane
            for idx, lane_polygon in enumerate(lane_polygons):
                curr_ego_pos = Point(curr_ego_state["x_meters"], curr_ego_state["y_meters"])
                if lane_polygon.contains(curr_ego_pos):
                    curr_ego_lane = idx
                    break
                curr_ego_lane = -1
            if prev_ego_lane != curr_ego_lane:
                if curr_ego_lane != -1 and prev_ego_lane != -1:
                    lane_keeping += 1
            
            # lane centering check
            if curr_ego_lane != -1:
                curr_ego_pos = Point(curr_ego_state["x_meters"], curr_ego_state["y_meters"])
                lane_polygon = lane_polygons[curr_ego_lane]
                lane_centering_polygon = lane_polygon.buffer(-lane_centering_buffer)
                if not lane_centering_polygon.contains(curr_ego_pos):
                    if not shapely.intersects(curr_ego_polygon, intersection_polygon):
                        lane_centering += 1
                
            # speed limit check
            curr_ego_speed = sqrt(curr_ego_state["x_velocity_meters_per_second"] ** 2 + curr_ego_state["y_velocity_meters_per_second"] ** 2)
            if curr_ego_speed > speed_limit_threshold:
                speed_limit += 1
            #speed_limit = max(speed_limit, curr_ego_speed - speed_limit_threshold)
            
            # abrupt movement check
            
        # The current state is a vehicle
        elif curr_state["type"] == "vehicle":
            vehicle_number_of_states += 1
            curr_vehicle_polygon = Polygon(curr_state["footprint"])
            
            # collision check
            if shapely.intersects(curr_ego_polygon, curr_vehicle_polygon):
                vehicle_collision += 1
            
            # clearance check
            dist = shapely.distance(curr_ego_polygon, curr_vehicle_polygon)
            if dist < vehicle_clearance_threshold:
                vehicle_clearance += 1
            #vehicle_clearance = max(vehicle_clearance, vehicle_clearance_threshold - dist)
            
            # time to collision check
            curr_vehicle_pos = Point(curr_state["x_meters"], curr_state["y_meters"])
            curr_ego_pos = Point(curr_ego_state["x_meters"], curr_ego_state["y_meters"])
            curr_ego_velocity_x = curr_ego_state["x_velocity_meters_per_second"]
            curr_ego_velocity_y = curr_ego_state["y_velocity_meters_per_second"]
            relative_position_x = curr_vehicle_pos.x - curr_ego_pos.x
            relative_position_y = curr_vehicle_pos.y - curr_ego_pos.y
            relative_position_norm = np.sqrt(relative_position_x ** 2 + relative_position_y ** 2)
            if relative_position_norm > 0:
                projected_speed = (curr_ego_velocity_x * relative_position_x + curr_ego_velocity_y * relative_position_y) / relative_position_norm
            else:
                projected_speed = 0
            if projected_speed > 0:
                time_to_collision = relative_position_norm / projected_speed
                if time_to_collision < vehicle_time_to_collision_threshold:
                    vehicle_time_to_collision += 1
        
        # The current state is a pedestrian
        elif curr_state["type"] == "pedestrian":
            ped_number_of_states += 1
            curr_pedestrian_polygon = Polygon(curr_state["footprint"])
            
            # collision check
            if shapely.intersects(curr_ego_polygon, curr_pedestrian_polygon):
                ped_collision += 1
            
            # clearance check
            dist = shapely.distance(curr_ego_polygon, curr_pedestrian_polygon)
            if dist < ped_clearance_threshold:
                ped_clearance += 1
            #ped_clearance = max(ped_clearance, ped_clearance_threshold - dist)
            
            # time to collision check
            curr_pedestrian_pos = Point(curr_state["x_meters"], curr_state["y_meters"])
            curr_ego_pos = Point(curr_ego_state["x_meters"], curr_ego_state["y_meters"])
            curr_ego_velocity_x = curr_ego_state["x_velocity_meters_per_second"]
            curr_ego_velocity_y = curr_ego_state["y_velocity_meters_per_second"]
            relative_position_x = curr_pedestrian_pos.x - curr_ego_pos.x
            relative_position_y = curr_pedestrian_pos.y - curr_ego_pos.y
            relative_position_norm = np.sqrt(relative_position_x ** 2 + relative_position_y ** 2)
            if relative_position_norm > 0:
                projected_speed = (curr_ego_velocity_x * relative_position_x + curr_ego_velocity_y * relative_position_y) / relative_position_norm
            else:
                projected_speed = 0
            if projected_speed > 0:
                time_to_collision = relative_position_norm / projected_speed
                if time_to_collision < ped_time_to_collision_threshold:
                    ped_time_to_collision += 1
    if to_print:
        print(f"Number of ego states: {ego_number_of_states}")
        print(f"Number of vehicle states: {vehicle_number_of_states}")
        print(f"Number of pedestrian states: {ped_number_of_states}")
        print(f"\nEvaluation results:")
        print(f"Pedestrian collision: {ped_collision}")
        print(f"Pedestrian time to collision: {ped_time_to_collision}")
        print(f"Pedestrian clearance: {ped_clearance}")
        print(f"Vehicle collision: {vehicle_collision}")
        print(f"Vehicle time to collision: {vehicle_time_to_collision}")
        print(f"Vehicle clearance: {vehicle_clearance}")
        print(f"Drivable region: {drivable_area}")
        print(f"Correct side of road: {correct_side_of_road}")
        print(f"Lane keeping: {lane_keeping}")
        print(f"Lane centering: {lane_centering}")
        print(f"Speed limit: {speed_limit}")
        print(f"\nMax heading movement: {min_value}")
        print(value_1, value_2, value_3, value_4)
    
    return [
        ped_collision,
        vehicle_collision,
        drivable_area,
        ped_time_to_collision,
        vehicle_time_to_collision,
        correct_side_of_road,
        ped_clearance,
        vehicle_clearance,
        lane_keeping,
        speed_limit,
        lane_centering
    ]

def plot_map(map_path, second_map_path=None, file_name="shapely_polygons.png"):
    gdf = gpd.read_file(map_path)
    polygons = [Polygon(geom) for geom in gdf['geometry']]
    if second_map_path:
        gdf2 = gpd.read_file(second_map_path)
        polygons += [Polygon(geom) for geom in gdf2['geometry']]
    
    labels = [f"{i}" for i in range(len(polygons))]

    fig, ax = plt.subplots(figsize=(8, 6))
    cmap = cm.get_cmap("tab10", len(polygons))

    # Plot each polygon
    for i, poly in enumerate(polygons):
        x, y = poly.exterior.xy
        ax.fill(x, y, color=cmap(i), alpha=0.6, edgecolor='black')
        
        # Add label at centroid
        centroid = poly.centroid
        ax.text(centroid.x, centroid.y, labels[i], ha='center', va='center', fontsize=10, color='black')

    # Set plot bounds
    all_x = [pt[0] for poly in polygons for pt in poly.exterior.coords]
    all_y = [pt[1] for poly in polygons for pt in poly.exterior.coords]
    ax.set_xlim(min(all_x) - 1, max(all_x) + 1)
    ax.set_ylim(min(all_y) - 1, max(all_y) + 1)
    ax.set_aspect('equal')
    ax.set_title("Polygons Visualization")

    # Save and show
    plt.savefig(file_name, dpi=300)
    plt.show()

if __name__ == "__main__":
    path_to_reasonable_crowd = "../../../Reasonable-Crowd"
    #traj_path = os.path.join(path_to_reasonable_crowd, "trajectories/U_1-a.json")
    traj_path = os.path.join(path_to_reasonable_crowd, "trajectories/U_46-a.json")
    boundary_path = os.path.join(path_to_reasonable_crowd, "maps/U_boundaries.gpkg")
    road_path = os.path.join(path_to_reasonable_crowd, "maps/U_road_segments.gpkg")
    intersection_path = os.path.join(path_to_reasonable_crowd, "maps/U_intersections.gpkg")
    lane_group_path = os.path.join(path_to_reasonable_crowd, "maps/U_lane_groups_polygons.gpkg")
    lane_path = os.path.join(path_to_reasonable_crowd, "maps/U_lanes_polygons.gpkg")
    
    evaluate_rules(traj_path, boundary_path, road_path, intersection_path, lane_group_path, lane_path, to_print=True)
    