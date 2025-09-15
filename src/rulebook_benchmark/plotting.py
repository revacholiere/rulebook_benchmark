import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib import animation
import numpy as np




def compare_realizations_gif(realization_model_pref, realization_human_pref, reason,
                             dpi=100, interval=100, margin=50):
    """
    Side-by-side animation comparing human vs model preferred realizations.
    Ego is highlighted in red, and the camera follows the ego in each subplot.
    """

    fig, axes = plt.subplots(1, 2, figsize=(12, 6), dpi=dpi)

    colors = {"Car": "blue", "Truck": "purple", "Pedestrian": "orange", "Bicycle": "green"}

    patches1, patches2 = [], []
    dummy = np.zeros((3, 2))  # minimal valid polygon

    # Lanes
    for lane in realization_human_pref.network.lanes:
        axes[0].add_patch(Polygon(lane.polygon.exterior.coords[:-1], closed=True,
                                  facecolor="lightgray", edgecolor="black", alpha=0.5))
    for lane in realization_model_pref.network.lanes:
        axes[1].add_patch(Polygon(lane.polygon.exterior.coords[:-1], closed=True,
                                  facecolor="lightgray", edgecolor="black", alpha=0.5))

    # Objects → Human pref
    for obj in realization_human_pref.objects:
        facecolor = "red" if obj is realization_human_pref.ego else colors.get(obj.object_type, "gray")
        poly = Polygon(dummy, closed=True, facecolor=facecolor, alpha=0.6)
        axes[0].add_patch(poly)
        patches1.append(poly)

    # Objects → Model pref
    for obj in realization_model_pref.objects:
        facecolor = "red" if obj is realization_model_pref.ego else colors.get(obj.object_type, "gray")
        poly = Polygon(dummy, closed=True, facecolor=facecolor, alpha=0.6)
        axes[1].add_patch(poly)
        patches2.append(poly)

    # Titles
    axes[0].set_title("Human Preference")
    axes[1].set_title("Model Preference - Reason:" + reason)
    for ax in axes:
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.axis('off')


    def init():
        for patch in patches1 + patches2:
            patch.set_xy(dummy)
        return patches1 + patches2

    def update(frame):
        # Human pref
        ws1 = realization_human_pref.get_world_state(min(frame, len(realization_human_pref) - 1))
        ego1 = ws1.ego_state
        for patch, state in zip(patches1, ws1.states):
            patch.set_xy(state.polygon.exterior.coords[:-1])
        cx1, cy1 = ego1.position
        axes[0].set_xlim(cx1 - margin, cx1 + margin)
        axes[0].set_ylim(cy1 - margin, cy1 + margin)

        # Model pref
        ws2 = realization_model_pref.get_world_state(min(frame, len(realization_model_pref) - 1))
        ego2 = ws2.ego_state
        for patch, state in zip(patches2, ws2.states):
            patch.set_xy(state.polygon.exterior.coords[:-1])
        cx2, cy2 = ego2.position
        axes[1].set_xlim(cx2 - margin, cx2 + margin)
        axes[1].set_ylim(cy2 - margin, cy2 + margin)

        return patches1 + patches2

    max_frames = max(len(realization_human_pref), len(realization_model_pref))
    anim = animation.FuncAnimation(fig, update, frames=max_frames, init_func=init,
                                   interval=interval, blit=True)
    return anim