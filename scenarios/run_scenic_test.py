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
    scenario = scenic.scenarioFromFile(file_path, model="scenic.simulators.metadrive.model", mode2D=True)
    scene, _ = scenario.generate()
    simulator = MetaDriveSimulator(sumo_map='../maps/Town05.net.xml')
    simulation = simulator.simulate(scene, maxSteps=max_steps, maxIterations=maxIterations)
    if not simulation:
        raise RuntimeError("Simulation failed.")
    return simulation

def visualize_simulation(simulation, ids, save_path='trajactory.mp4', fps=10, trail_length=15):
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
    
if __name__ == "__main__":
    ids = ['egoPoly', 'advPoly', 'bicyclePoly']
    simulation = run_metadrive_scenario("crash_waymo-august-12-2019/crash_waymo-august-12-2019.scenic", max_steps=MAX_STEPS, seed=123)
    #ids = ['egoPoly', 'adv1Poly', 'adv2Poly', 'adv3Poly']
    #simulation = run_metadrive_scenario("multi_02/multi_02.scenic", max_steps=MAX_STEPS, seed=123)
    visualize_simulation(simulation, ids)
