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

def animate_realization(realization, dpi=100, interval=100, margin=50):
    """
    Single realization animation.
    Ego is highlighted in red, and the camera follows the ego.
    Draws an arrow for vehicle heading using object.orientation.yaw.
    Also writes the heading (yaw in radians) next to the arrow.
    """

    fig, ax = plt.subplots(figsize=(6, 6), dpi=dpi)

    colors = {"Car": "blue", "Truck": "purple", "Pedestrian": "orange", "Bicycle": "green"}
    patches = []
    arrows = []
    texts = []
    dummy = np.zeros((3, 2))  # minimal valid polygon

    # Lanes
    for lane in realization.network.lanes:
        ax.add_patch(Polygon(lane.polygon.exterior.coords[:-1], closed=True,
                                facecolor="lightgray", edgecolor="black", alpha=0.5))

    # Objects
    for obj in realization.objects:
        facecolor = "red" if obj is realization.ego else colors.get(obj.object_type, "gray")
        poly = Polygon(dummy, closed=True, facecolor=facecolor, alpha=0.6)
        ax.add_patch(poly)
        patches.append(poly)
        # Arrow for heading
        arrow = ax.arrow(0, 0, 0, 0, head_width=2, head_length=4, fc=facecolor, ec=facecolor)
        arrows.append(arrow)
        # Text for heading
        text = ax.text(0, 0, "", fontsize=8, color=facecolor)
        texts.append(text)

    # Title and formatting
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.axis('off')

    def init():
        for patch in patches:
            patch.set_xy(dummy)
        for arrow in arrows:
            arrow.set_visible(False)
        for text in texts:
            text.set_text("")
            text.set_visible(False)
        return patches + arrows + texts

    def update(frame):
        ws = realization.get_world_state(min(frame, len(realization) - 1))
        ego = ws.ego_state
        for i, (patch, state, arrow, text) in enumerate(zip(patches, ws.states, arrows, texts)):
            patch.set_xy(state.polygon.exterior.coords[:-1])
            # Draw heading arrow
            x, y = state.position
            yaw = getattr(state.orientation, "yaw", 0)
            dx = 8 * np.cos(yaw)
            dy = 8 * np.sin(yaw)
            arrow.remove()
            facecolor = "red" if realization.objects[i] is realization.ego else colors.get(realization.objects[i].object_type, "gray")
            arrows[i] = ax.arrow(x, y, dx, dy, head_width=2, head_length=4, fc=facecolor, ec=facecolor)
            # Heading text (in radians)
            text.set_text(f"{yaw:.2f} rad")
            text.set_position((x + dx + 2, y + dy + 2))
            text.set_color(facecolor)
            text.set_visible(True)
        cx, cy = ego.position
        ax.set_xlim(cx - margin, cx + margin)
        ax.set_ylim(cy - margin, cy + margin)
        return patches + arrows + texts

    max_frames = len(realization)
    anim = animation.FuncAnimation(fig, update, frames=max_frames, init_func=init,
                                    interval=interval, blit=True)
    return anim
