import math
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
import numpy as np
from matplotlib import transforms


def alignment_figure(aggregate, path):
    alignment = [a for a in aggregate if a["case"] == "free_boids" and "alignment" in a["parameters"]]
    if not alignment:
        return False
    fig, ax = plt.subplots(figsize=(7, 4))
    for item in alignment:
        x = item["parameters"]["alignment"]
        for value in item["seed_values"]:
            if value["polarization"] is not None:
                ax.scatter(x, value["polarization"], color="steelblue", alpha=.6)
        if item["polarization_mean"] is not None:
            ax.errorbar(x, item["polarization_mean"], yerr=item["polarization_sd"] or 0, fmt="o", color="black", capsize=4)
    ax.set(xlabel="Alignment weight", ylabel="Final polarization", title="Seed values, mean and sample SD")
    ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return True


def figures(frames, rows, directory):
    directory = Path(directory); directory.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 7)); last = frames[-1]
    for rect in last["scene"]["rectangles"]:
        x,y = rect["center"]; w,h = rect["width"], rect["height"]
        patch = Rectangle((x-w/2,y-h/2),w,h,facecolor="gray",alpha=.4,edgecolor="black")
        patch.set_transform(transforms.Affine2D().rotate_deg_around(x,y,rect["angle_degrees"]) + ax.transData); ax.add_patch(patch)
    for kind, color in (("wind", "#4aa8d8"), ("noise", "#ac79ca")):
        for region in last["scene"][kind]:
            x,y=region["center"]; w,h=region["length"],region["width"]
            patch=Rectangle((x-w/2,y-h/2),w,h,facecolor=color,alpha=.12,edgecolor=color)
            patch.set_transform(transforms.Affine2D().rotate_deg_around(x,y,region["angle_degrees"]) + ax.transData); ax.add_patch(patch)
    groups={g["uid"]:g for g in last["groups"]}
    for g in groups.values():
        cmd=g["command"]
        if cmd["type"] == "gather": ax.add_patch(Circle(cmd["center"],cmd["radius"], fill=False, linestyle="--"))
        if cmd["type"] == "route":
            pts=cmd["points"]; radius=cmd["radius"]
            for point in pts: ax.add_patch(Circle(point,radius,facecolor="steelblue",alpha=.12,edgecolor="none"))
            for a,b in zip(pts,pts[1:]):
                a=np.array(a,dtype=float); b=np.array(b,dtype=float); d=b-a; length=np.linalg.norm(d)
                if length:
                    offset=np.array([-d[1],d[0]]) * radius / length
                    ax.add_patch(Polygon([a+offset,b+offset,b-offset,a-offset],facecolor="steelblue",alpha=.12,edgecolor="none"))
            ax.plot([p[0] for p in pts],[p[1] for p in pts], linestyle="--",alpha=.5)
    tracks={}
    for frame in frames[::max(1, len(frames)//1000)]:
        for a in frame["agents"]:
            if a["state"] != "terminated": tracks.setdefault(a["uid"], []).append(a["position"])
    for a in last["agents"]:
        pts=tracks.get(a["uid"],[])+[a["position"]]
        if not pts: continue
        color=tuple(v/255 for v in groups[a["group_uid"]]["rgba"])
        ax.plot([p[0] for p in pts],[p[1] for p in pts],color=color,alpha=.6,linewidth=.8)
        ax.scatter(*pts[0],color=color,s=8,marker="o"); ax.scatter(*pts[-1],color=color,s=16,marker="x" if a["state"] == "terminated" else ".")
        if len(last["agents"]) <= 50 and a["state"] != "terminated":
            ax.add_patch(Circle(a["position"],groups[a["group_uid"]]["config"]["body_radius"],fill=False,edgecolor=color,linewidth=.6))
    ax.set(xlabel="x (m)",ylabel="y (m)",title="Trajectories (cross: terminated)"); ax.set_aspect("equal",adjustable="datalim"); ax.grid(alpha=.2); fig.tight_layout(); fig.savefig(directory/"trajectory.png",dpi=140); plt.close(fig)
    fig, axes=plt.subplots(5,1,figsize=(9,12),sharex=True)
    for uid in groups:
        data=[r for r in rows if r["group_uid"]==uid]; t=[r["time"] for r in data]
        for state in ("moving","arrived","terminated"): axes[0].plot(t,[r[state] for r in data],label=f"G{uid} {state}")
        for ax,key in zip(axes[1:4],("polarization","spread","mean_speed")): ax.plot(t,[r[key] if r[key] is not None else math.nan for r in data],label=f"G{uid}")
        for key in ("target_distance","route_error_median"):
            if any(r[key] is not None for r in data): axes[4].plot(t,[r[key] if r[key] is not None else math.nan for r in data],label=f"G{uid} {key}")
    for ax,title in zip(axes,("State fractions", "Polarization", "RMS spread (m)", "Ground speed (m/s)", "Task distance (m)")):
        ax.set_ylabel(title); ax.grid(alpha=.2)
        if ax.get_legend_handles_labels()[0]: ax.legend(fontsize=7)
    axes[-1].set_xlabel("Time (s)"); fig.tight_layout(); fig.savefig(directory/"dynamics.png",dpi=140); plt.close(fig)
