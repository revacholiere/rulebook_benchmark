import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))

import scenic
import random
from scenic.simulators.metadrive import MetaDriveSimulator
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation, FFMpegWriter
from shapely.geometry import Polygon as ShapelyPolygon

MAX_STEPS = 100

def run_metadrive_scenario(file_path, max_steps=100, seed=None, maxIterations=10):
    if seed is not None:
        random.seed(seed)
    scenic.setDebuggingOptions(verbosity=1, fullBacktrace=True, debugExceptions=False, debugRejections=False)
    scenario = scenic.scenarioFromFile(file_path, model="scenic.simulators.metadrive.model", mode2D=True)
    scene, _ = scenario.generate()
    simulator = MetaDriveSimulator(sumo_map='../maps/Town05.net.xml')
    simulation = simulator.simulate(scene, maxSteps=max_steps, maxIterations=maxIterations)
    if not simulation:
        raise RuntimeError("Simulation failed.")
    return simulation

def visualize_simulation(simulation, ids, save_path='trajectory.mp4', fps=10, trail_length=15):
    trajectories = {}
    for id in ids:
        if id not in simulation.records:
            print(f"ID {id} not found in simulation records.")
            return
        trajectories[id] = simulation.records[id]
        
    # Number of frames = max length across vehicles
    num_frames = max(len(traj) for traj in trajectories.values())

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal', 'box')

    # Create polygon patches and text labels for each vehicle
    vehicle_patches = {}
    vehicle_labels = {}
    for i, vid in enumerate(trajectories):
        poly = trajectories[vid][0][1]
        
        if 'Lane' in str(vid):  # Lane polygon
            facecolor = plt.cm.Pastel1(i % 9)   # softer colormap
            alpha = 0.2                         # more transparent
        else:  # Vehicle polygon
            facecolor = plt.cm.tab10(i % 10)    # bold colormap
            alpha = 0.5
            
        patch = Polygon(list(poly.exterior.coords), closed=True, 
                        facecolor=facecolor, alpha=alpha, edgecolor='black')
        ax.add_patch(patch)
        vehicle_patches[vid] = patch

        if 'Lane' not in str(vid):
            label = ax.text(poly.centroid.x, poly.centroid.y, str(vid).split("Poly")[0], 
                            fontsize=10, ha="center", va="center", color="black", weight="bold")
            vehicle_labels[vid] = label
        else:
            vehicle_labels[vid] = ax.text(poly.centroid.x, poly.centroid.y, '', 
                                          fontsize=10, ha="center", va="center", color="black", weight="bold")

    # Determine global plot limits
    all_x, all_y = [], []
    for traj in trajectories.values():
        for poly in traj:
            poly = poly[1]
            x, y = poly.exterior.xy
            all_x.extend(x)
            all_y.extend(y)
    ax.set_xlim(min(all_x) - 5, max(all_x) + 5)
    ax.set_ylim(min(all_y) - 5, max(all_y) + 5)

    # For fading trails: store line objects per vehicle
    vehicle_trails = {vid: [] for vid in trajectories}

    def update(frame):
        artists = []
        for i, (vid, traj) in enumerate(trajectories.items()):
            if frame < len(traj):
                poly: ShapelyPolygon = traj[frame][1]
                # Update polygon
                vehicle_patches[vid].set_xy(list(poly.exterior.coords))
                artists.append(vehicle_patches[vid])

                # Update label
                centroid = poly.centroid
                vehicle_labels[vid].set_position((centroid.x, centroid.y))
                artists.append(vehicle_labels[vid])

                # Add trail segment (centroid path)
                if frame > 0 and 'Lane' not in str(vid):  # Skip lane markings
                    prev = traj[frame-1][1].centroid
                    curr = centroid
                    line, = ax.plot([prev.x, curr.x], [prev.y, curr.y],
                                    color=plt.cm.tab10(i % 10), alpha=0.6, linewidth=2)
                    vehicle_trails[vid].append(line)

                    # Keep only the last `trail_length` segments
                    if len(vehicle_trails[vid]) > trail_length:
                        old_line = vehicle_trails[vid].pop(0)
                        old_line.remove()

                    # Update fading alpha
                    for j, l in enumerate(vehicle_trails[vid]):
                        l.set_alpha((j+1) / trail_length)

                    artists.extend(vehicle_trails[vid])
        return artists

    ani = FuncAnimation(fig, update, frames=num_frames, blit=True, interval=1000/fps, repeat=False)

    # Save as mp4 using ffmpeg
    writer = FFMpegWriter(fps=fps, codec="libx264", bitrate=-1)
    ani.save(save_path, writer=writer)
    plt.close(fig)
    print(f"Video saved to {save_path}")

def visualize_simulation_points(simulation, save_path='trajectory.mp4', fps=10, trail_length=15):
    trajectories = simulation.trajectory  # [(pos1, pos2, ...), (pos1, pos2, ...), ...]
    num_frames = len(trajectories)

    # Determine global plot limits
    all_x, all_y = [], []
    for frame in trajectories:
        for pos in frame:
            all_x.append(pos.x)
            all_y.append(pos.y)
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal', 'box')
    ax.set_xlim(min(all_x) - 5, max(all_x) + 5)
    ax.set_ylim(min(all_y) - 5, max(all_y) + 5)

    # Initialize storage for all objects seen so far
    scatters = {}
    trails = {}
    colors = plt.cm.tab10.colors  # 10 distinct colors

    def update(frame_idx):
        artists = []
        positions = trajectories[frame_idx]
        num_objects = len(positions)

        # Ensure we have scatter + trail for each object seen so far
        for i in range(num_objects):
            if i not in scatters:
                color = colors[i % len(colors)]
                scatters[i] = ax.plot([], [], 'o', color=color, markersize=6)[0]
                trails[i] = []

            pos = positions[i]
            x, y = pos.x, pos.y
            scatters[i].set_data([x], [y])
            artists.append(scatters[i])

            # Add trail
            if frame_idx > 0 and i < len(trajectories[frame_idx - 1]):
                prev = trajectories[frame_idx - 1][i]
                line, = ax.plot([prev.x, x], [prev.y, y],
                                color=colors[i % len(colors)], alpha=0.6, linewidth=2)
                trails[i].append(line)

                # Keep only the last `trail_length` segments
                if len(trails[i]) > trail_length:
                    old_line = trails[i].pop(0)
                    old_line.remove()

                # Fade older trail segments
                for j, l in enumerate(trails[i]):
                    l.set_alpha((j + 1) / trail_length)

                artists.extend(trails[i])

        # Handle objects that disappeared
        for i in list(scatters.keys()):
            if i >= num_objects:  # object no longer in this frame
                scatters[i].set_data([], [])
                # optionally remove trails too
                for line in trails[i]:
                    line.remove()
                trails[i] = []
                artists.append(scatters[i])

        return artists

    ani = FuncAnimation(fig, update, frames=num_frames, blit=True, interval=1000 / fps, repeat=False)

    writer = FFMpegWriter(fps=fps, codec="libx264", bitrate=-1)
    ani.save(save_path, writer=writer)
    plt.close(fig)
    print(f"Video saved to {save_path}")
    
if __name__ == "__main__":
    #ids = ['egoPoly', 'advPoly', 'bicyclePoly']
    #simulation = run_metadrive_scenario("crash_waymo-august-12-2019/crash_waymo-august-12-2019.scenic", max_steps=MAX_STEPS, seed=123)
    ids = ['egoPoly', 'trailingCarPoly', 'trailingCar2Poly']#, 'parkedCar2Poly']
    simulation = run_metadrive_scenario("basic/basic.scenic", max_steps=MAX_STEPS, seed=123)
    print(len(simulation.trajectory), len(simulation.trajectory[0]))
    print(simulation.trajectory[0], simulation.trajectory[1], simulation.trajectory[51], simulation.trajectory[99])
    #visualize_simulation(simulation, ids)
    visualize_simulation_points(simulation)
