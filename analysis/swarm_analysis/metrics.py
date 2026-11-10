import json
from collections import Counter
from itertools import groupby
import numpy as np


def arrival_thresholds(initial, events):
    """First completed tick with >=90% arrived, including ticks between snapshots."""
    agents = {a["uid"]: a for a in initial["agents"]}
    sizes = Counter(a["group_uid"] for a in agents.values())
    counts = Counter(a["group_uid"] for a in agents.values() if a["state"] == "arrived")
    applicable = {g["uid"] for g in initial["groups"] if g["command"]["type"] in ("gather", "route")}
    result = {uid: 0 for uid in applicable if sizes[uid] and counts[uid] / sizes[uid] >= .9}
    for step, batch in groupby(events, key=lambda e: e["step"]):
        for event in batch:
            if step and event["type"] == "state_changed":
                uid = agents[event["agent_uids"][0]]["group_uid"]
                counts[uid] += int(event["to"] == "arrived") - int(event["from"] == "arrived")
        for uid in applicable - result.keys():
            if sizes[uid] and counts[uid] / sizes[uid] >= .9:
                result[uid] = step
    return result


def route_distances(points, route):
    vertices = np.asarray(route, dtype=float)
    best = np.full(len(points), np.inf)
    if len(vertices) == 1:
        return np.linalg.norm(points - vertices[0], axis=1)
    for a, b in zip(vertices, vertices[1:]):
        d = b - a
        n = float(d @ d)
        t = np.zeros(len(points)) if n == 0 else np.clip((points - a) @ d / n, 0, 1)
        best = np.minimum(best, np.linalg.norm(points - (a + t[:, None] * d), axis=1))
    return best


def frame_metrics(frame, pair_limit=2_000_000):
    groups = {g["uid"]: g for g in frame["groups"]}
    live = [a for a in frame["agents"] if a["state"] != "terminated"]
    global_gap = None
    if 1 < len(live) and len(live) ** 2 <= pair_limit:
        p = np.array([a["position"] for a in live]); r = np.array([groups[a["group_uid"]]["config"]["body_radius"] for a in live])
        distances = np.linalg.norm(p[:, None] - p[None, :], axis=2) - r[:, None] - r[None, :]
        np.fill_diagonal(distances, np.inf); global_gap = float(distances.min())
    rows = []
    for uid, group in groups.items():
        agents = [a for a in frame["agents"] if a["group_uid"] == uid]
        alive = [a for a in agents if a["state"] != "terminated"]
        n = len(agents)
        row = {"step": frame["steps"], "time": frame["time"], "group_uid": uid, "n": n,
               **{s: sum(a["state"] == s for a in agents) / n if n else None for s in ("moving", "arrived", "terminated")},
               "centroid_x": None, "centroid_y": None, "spread": None, "polarization": None,
               "moving_vectors": 0, "mean_speed": None, "nearest_neighbor": None, "min_body_gap": global_gap,
               "target_distance": None, "inside_fraction": None, "route_error_median": None, "route_error_max": None, "corridor_excess_max": None}
        if alive:
            p = np.array([a["position"] for a in alive]); v = np.array([a["velocity"] for a in alive])
            center = p.mean(axis=0); speed = np.linalg.norm(v, axis=1); moving = speed > 1e-9
            row.update(centroid_x=float(center[0]), centroid_y=float(center[1]), spread=float(np.sqrt(np.mean(np.sum((p-center)**2, axis=1)))), mean_speed=float(speed.mean()), moving_vectors=int(moving.sum()))
            if moving.any(): row["polarization"] = float(np.linalg.norm((v[moving] / speed[moving, None]).mean(axis=0)))
            if len(p) > 1 and len(p)**2 <= pair_limit:
                d = np.linalg.norm(p[:, None] - p[None, :], axis=2); np.fill_diagonal(d, np.inf)
                row["nearest_neighbor"] = float(np.median(d.min(axis=1)))
            command = group["command"]
            if command["type"] == "gather":
                dist = np.linalg.norm(p - np.array(command["center"]), axis=1)
                row.update(target_distance=float(dist.mean()), inside_fraction=float((dist <= command["radius"]).sum()/n))
            if command["type"] == "route":
                dist = route_distances(p, command["points"])
                row.update(route_error_median=float(np.median(dist)), route_error_max=float(dist.max()), corridor_excess_max=float(np.maximum(0, dist-command["radius"]).max()))
        reasons = {}
        for key, value in row.items():
            if value is not None: continue
            if key == "min_body_gap":
                reasons[key] = "pair_limit" if len(live)**2 > pair_limit else "fewer_than_two_living_agents"
            elif key == "nearest_neighbor":
                reasons[key] = "pair_limit" if len(alive)**2 > pair_limit else "fewer_than_two_living_agents"
            elif key in ("target_distance", "inside_fraction") and group["command"]["type"] != "gather": reasons[key] = "not_applicable"
            elif key.startswith("route_") or key == "corridor_excess_max": reasons[key] = "not_applicable" if group["command"]["type"] != "route" else "no_living_agents"
            elif key == "polarization": reasons[key] = "no_moving_living_agents"
            else: reasons[key] = "no_living_agents"
        row["undefined_reasons"] = json.dumps(reasons, sort_keys=True)
        row["min_body_gap_scope"] = "world"
        rows.append(row)
    return rows
