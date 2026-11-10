import numpy as np


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
        rows.append(row)
    return rows
