import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib import animation
import numpy as np


def animate_realization(realization, dpi=100, interval=100, margin=50):
    fig, ax = plt.subplots(figsize=(6, 6), dpi=dpi)

    colors = {"Car": "blue", "Truck": "purple", "Pedestrian": "orange", "Bicycle": "green"}
    patches, arrows, texts = [], [], []
    dummy = np.zeros((3, 2))

    # Lane base layer
    for lane in realization.network.lanes:
        poly = Polygon(lane.polygon.exterior.coords[:-1], closed=True,
                       facecolor="lightgray", edgecolor="black", alpha=0.5)
        ax.add_patch(poly)

    ego_lane_patch = Polygon(dummy, closed=True, facecolor="yellow", alpha=0.3)
    ax.add_patch(ego_lane_patch)

    for obj in realization.objects:
        facecolor = "red" if obj is realization.ego else colors.get(obj.object_type, "gray")
        poly = Polygon(dummy, closed=True, facecolor=facecolor, alpha=0.6)
        ax.add_patch(poly)
        patches.append(poly)
        arrows.append([ax.arrow(0, 0, 0, 0, head_width=2, head_length=4, fc=facecolor, ec=facecolor)])
        texts.append(ax.text(0, 0, "", fontsize=8, color=facecolor))

    lane_arrow = [ax.arrow(0, 0, 0, 0, head_width=2, head_length=4, fc="yellow", ec="yellow")]

    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.axis('off')

    def init():
        for patch in patches:
            patch.set_xy(dummy)
        ego_lane_patch.set_xy(dummy)
        for a in arrows:
            a[0].set_visible(False)
        for t in texts:
            t.set_visible(False)
        lane_arrow[0].set_visible(False)
        return patches + [ego_lane_patch] + [a[0] for a in arrows] + texts + [lane_arrow[0]]

    def update(frame):
        ws = realization.get_world_state(min(frame, len(realization) - 1))
        ego = ws.ego_state
        ego_pos = ego.position
        ego_yaw = ego.orientation.yaw

        lane = getattr(ego, "lane", None)
        if lane is not None:
            ego_lane_patch.set_xy(lane.polygon.exterior.coords[:-1])
            lane_yaw = lane.orientation.value(ego_pos)
            lane_arrow[0].remove()
            lane_dx, lane_dy = 8 * np.cos(lane_yaw), 8 * np.sin(lane_yaw)
            lane_arrow[0] = ax.arrow(ego_pos[0], ego_pos[1], lane_dx, lane_dy,
                                     head_width=2, head_length=4, fc="yellow", ec="yellow")
        else:
            ego_lane_patch.set_xy(dummy)
            lane_arrow[0].set_visible(False)

        for i, (patch, state) in enumerate(zip(patches, ws.states)):
            patch.set_xy(state.polygon.exterior.coords[:-1])
            x, y = state.position
            yaw = state.orientation.yaw
            dx, dy = 8 * np.cos(yaw), 8 * np.sin(yaw)
            arrows[i][0].remove()
            color = "red" if realization.objects[i] is realization.ego else colors.get(realization.objects[i].object_type, "gray")
            arrows[i][0] = ax.arrow(x, y, dx, dy, head_width=2, head_length=4, fc=color, ec=color)
            texts[i].set_text(f"{yaw:.2f} rad")
            texts[i].set_position((x + dx + 2, y + dy + 2))
            texts[i].set_color(color)
            texts[i].set_visible(True)

        cx, cy = ego_pos
        ax.set_xlim(cx - margin, cx + margin)
        ax.set_ylim(cy - margin, cy + margin)
        return patches + [ego_lane_patch] + [a[0] for a in arrows] + texts + [lane_arrow[0]]

    max_frames = len(realization)
    anim = animation.FuncAnimation(fig, update, frames=max_frames, init_func=init,
                                   interval=interval, blit=True)
    return anim


def compare_realizations_gif(realization_model_pref, realization_human_pref, reason, agreement,
                             dpi=100, interval=100, margin=50):
    fig, axes = plt.subplots(1, 2, figsize=(12, 6), dpi=dpi)
    colors = {"Car": "blue", "Truck": "purple", "Pedestrian": "orange", "Bicycle": "green"}
    patches1, patches2 = [], []
    dummy = np.zeros((3, 2))

    for lane in realization_human_pref.network.lanes:
        axes[0].add_patch(Polygon(lane.polygon.exterior.coords[:-1], closed=True,
                                  facecolor="lightgray", edgecolor="black", alpha=0.5))
    for lane in realization_model_pref.network.lanes:
        axes[1].add_patch(Polygon(lane.polygon.exterior.coords[:-1], closed=True,
                                  facecolor="lightgray", edgecolor="black", alpha=0.5))

    ego_lane_patch1 = Polygon(dummy, closed=True, facecolor="yellow", alpha=0.3)
    ego_lane_patch2 = Polygon(dummy, closed=True, facecolor="yellow", alpha=0.3)
    axes[0].add_patch(ego_lane_patch1)
    axes[1].add_patch(ego_lane_patch2)

    for obj in realization_human_pref.objects:
        color = "red" if obj is realization_human_pref.ego else colors.get(obj.object_type, "gray")
        poly = Polygon(dummy, closed=True, facecolor=color, alpha=0.6)
        axes[0].add_patch(poly)
        patches1.append(poly)
    for obj in realization_model_pref.objects:
        color = "red" if obj is realization_model_pref.ego else colors.get(obj.object_type, "gray")
        poly = Polygon(dummy, closed=True, facecolor=color, alpha=0.6)
        axes[1].add_patch(poly)
        patches2.append(poly)

    ego_arrow1 = [axes[0].arrow(0, 0, 0, 0, head_width=2, head_length=4, fc="red", ec="red")]
    ego_arrow2 = [axes[1].arrow(0, 0, 0, 0, head_width=2, head_length=4, fc="red", ec="red")]
    lane_arrow1 = [axes[0].arrow(0, 0, 0, 0, head_width=2, head_length=4, fc="yellow", ec="yellow")]
    lane_arrow2 = [axes[1].arrow(0, 0, 0, 0, head_width=2, head_length=4, fc="yellow", ec="yellow")]

    text1 = axes[0].text(0, 0, "", fontsize=8, color="red")
    text2 = axes[1].text(0, 0, "", fontsize=8, color="red")

    axes[0].set_title("Human Preference - Agreement: " + str(agreement))
    axes[1].set_title("Model Preference - Reason: " + reason)
    for ax in axes:
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.axis('off')

    def init():
        for patch in patches1 + patches2:
            patch.set_xy(dummy)
        ego_lane_patch1.set_xy(dummy)
        ego_lane_patch2.set_xy(dummy)
        return patches1 + patches2 + [ego_lane_patch1, ego_lane_patch2,
                                      ego_arrow1[0], ego_arrow2[0],
                                      lane_arrow1[0], lane_arrow2[0],
                                      text1, text2]

    def update(frame):
        # Human side
        ws1 = realization_human_pref.get_world_state(min(frame, len(realization_human_pref) - 1))
        ego1 = ws1.ego_state
        ego_pos1 = ego1.position
        ego_yaw1 = ego1.orientation.yaw
        lane1 = getattr(ego1, "lane", None)
        if lane1 is not None:
            ego_lane_patch1.set_xy(lane1.polygon.exterior.coords[:-1])
            lane_yaw1 = lane1.orientation.value(ego_pos1)
            lane_arrow1[0].remove()
            ldx1, ldy1 = 8 * np.cos(lane_yaw1), 8 * np.sin(lane_yaw1)
            lane_arrow1[0] = axes[0].arrow(ego_pos1[0], ego_pos1[1], ldx1, ldy1,
                                           head_width=2, head_length=4, fc="yellow", ec="yellow")
        else:
            ego_lane_patch1.set_xy(dummy)
            lane_arrow1[0].set_visible(False)

        for patch, state in zip(patches1, ws1.states):
            patch.set_xy(state.polygon.exterior.coords[:-1])

        ego_arrow1[0].remove()
        dx1, dy1 = 8 * np.cos(ego_yaw1), 8 * np.sin(ego_yaw1)
        ego_arrow1[0] = axes[0].arrow(ego_pos1[0], ego_pos1[1], dx1, dy1,
                                      head_width=2, head_length=4, fc="red", ec="red")
        text1.set_text(f"{ego_yaw1:.2f} rad")
        text1.set_position((ego_pos1[0] + dx1 + 2, ego_pos1[1] + dy1 + 2))
        axes[0].set_xlim(ego_pos1[0] - margin, ego_pos1[0] + margin)
        axes[0].set_ylim(ego_pos1[1] - margin, ego_pos1[1] + margin)

        # Model side
        ws2 = realization_model_pref.get_world_state(min(frame, len(realization_model_pref) - 1))
        ego2 = ws2.ego_state
        ego_pos2 = ego2.position
        ego_yaw2 = ego2.orientation.yaw
        lane2 = getattr(ego2, "lane", None)
        if lane2 is not None:
            ego_lane_patch2.set_xy(lane2.polygon.exterior.coords[:-1])
            lane_yaw2 = lane2.orientation.value(ego_pos2)
            lane_arrow2[0].remove()
            ldx2, ldy2 = 8 * np.cos(lane_yaw2), 8 * np.sin(lane_yaw2)
            lane_arrow2[0] = axes[1].arrow(ego_pos2[0], ego_pos2[1], ldx2, ldy2,
                                           head_width=2, head_length=4, fc="yellow", ec="yellow")
        else:
            ego_lane_patch2.set_xy(dummy)
            lane_arrow2[0].set_visible(False)

        for patch, state in zip(patches2, ws2.states):
            patch.set_xy(state.polygon.exterior.coords[:-1])

        ego_arrow2[0].remove()
        dx2, dy2 = 8 * np.cos(ego_yaw2), 8 * np.sin(ego_yaw2)
        ego_arrow2[0] = axes[1].arrow(ego_pos2[0], ego_pos2[1], dx2, dy2,
                                      head_width=2, head_length=4, fc="red", ec="red")
        text2.set_text(f"{ego_yaw2:.2f} rad")
        text2.set_position((ego_pos2[0] + dx2 + 2, ego_pos2[1] + dy2 + 2))
        axes[1].set_xlim(ego_pos2[0] - margin, ego_pos2[0] + margin)
        axes[1].set_ylim(ego_pos2[1] - margin, ego_pos2[1] + margin)

        return patches1 + patches2 + [ego_lane_patch1, ego_lane_patch2,
                                      ego_arrow1[0], ego_arrow2[0],
                                      lane_arrow1[0], lane_arrow2[0],
                                      text1, text2]

    max_frames = max(len(realization_human_pref), len(realization_model_pref))
    anim = animation.FuncAnimation(fig, update, frames=max_frames, init_func=init,
                                   interval=interval, blit=True)
    return anim