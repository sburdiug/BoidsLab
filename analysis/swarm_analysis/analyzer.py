"""Stream all observations to CSV; retain only bounded frames for figures."""
import csv
import json
import math
import time
from pathlib import Path
from .reader import iter_run, strict_json, option
from .metrics import frame_metrics, arrival_thresholds
from .plots import figures

ANALYSIS_VERSION = "3"


def write_json(path, data):
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")
    temp.replace(path)


def analyze(path, expected_steps=None, component_radius=None):
    path = Path(path)
    meta_path = path / "metadata.json"
    has_metadata = meta_path.exists()
    if component_radius is not None and (isinstance(component_radius, bool) or not isinstance(component_radius, (int, float)) or not math.isfinite(component_radius) or component_radius <= 0):
        raise ValueError("component radius must be positive and finite")
    meta = {}
    # Invalidate old derived results before validation, including failed reanalysis.
    write_json(path / "validation.json", {"valid": False, "status": "analyzing"})
    for name in ("summary.json", "metrics.csv", "plots/trajectory.png", "plots/dynamics.png", "plots/cohesion.png"):
        (path / name).unlink(missing_ok=True)
    start = time.perf_counter()
    phase = "validation"
    warnings = set()
    frames, plot_rows, groups = [], [], {}
    count = 0
    try:
        meta = strict_json(meta_path.read_text(encoding="utf-8")) if has_metadata else {}
        if component_radius is None:
            component_radius = meta.get("analysis_settings", {}).get("component_radius_m", 1.0)
        if isinstance(component_radius, bool) or not isinstance(component_radius, (int, float)) or not math.isfinite(component_radius) or component_radius <= 0:
            raise ValueError("component radius must be positive and finite")
        meta["analysis_settings"] = {"component_radius_m": component_radius}
        if expected_steps is None and has_metadata:
            expected_steps = option(meta["argv"], "--steps", 100)
        # shortcut: standalone traces without metadata use their final summary's horizon.
        horizon = expected_steps if expected_steps is not None else strict_json((path / "stdout.json").read_text())["steps"]
        interval = option(meta.get("argv", []), "--trace-every", 1)
        stride = max(1, math.ceil((math.ceil(horizon / interval) + 1) / 999))
        if not has_metadata:
            warnings.add("No process metadata: requested horizon and process success cannot be independently verified.")
        if interval > 1:
            warnings.add("Sparse snapshots: distances are sampled; arrival thresholds use all state events at simulation-step resolution.")
        with (path / "metrics.csv.tmp").open("w", encoding="utf-8", newline="") as output:
            writer = None
            source = iter(iter_run(path, expected_steps, interval))
            while True:
                phase = "validation"
                try:
                    frame = next(source)
                except StopIteration:
                    break
                phase = "analysis"
                if count == 0:
                    frame_budget = max(2, min(1000, 100_000 // max(1, len(frame["agents"])) ))
                    stride = max(1, math.ceil((math.ceil(horizon / interval) + 1) / (frame_budget - 1)))
                rows = frame_metrics(frame, link_distance=component_radius)
                commands = {g["uid"]: g["command"]["type"] for g in frame["groups"]}
                for row in rows:
                    row.update(run_hash=meta.get("run_hash", "unknown"), case=meta.get("case", "standalone"),
                               parameters=json.dumps(meta.get("params", {}), sort_keys=True),
                               analysis_version=ANALYSIS_VERSION, length_unit="m", time_unit="s", speed_unit="m/s")
                    for metric, reason in json.loads(row["undefined_reasons"]).items():
                        if reason == "pair_limit":
                            warnings.add(f"{metric} skipped when living population squared exceeds 2000000 (pair_limit).")
                    uid = str(row["group_uid"])
                    applicable = commands[row["group_uid"]] in ("gather", "route")
                    group = groups.setdefault(uid, {"command": commands[row["group_uid"]], "time_to_90_arrived": None,
                        "time_to_90_interval": None, "time_to_90_reason": "not_reached_by_horizon" if applicable else "not_applicable",
                        "max_terminated": None, "minimum_sampled_world_body_gap": None})
                    for dest, key, fn in (("max_terminated", "terminated", max), ("minimum_sampled_world_body_gap", "min_body_gap", min)):
                        if row[key] is not None:
                            group[dest] = row[key] if group[dest] is None else fn(group[dest], row[key])
                    group["final"] = row
                if rows:
                    if writer is None:
                        writer = csv.DictWriter(output, fieldnames=rows[0].keys())
                        writer.writeheader()
                    writer.writerows(rows)
                if count % stride == 0 or frame["steps"] == horizon:
                    frames.append({k: v for k, v in frame.items() if k != "events"})
                    plot_rows.extend(rows)
                count += 1
        phase = "analysis"
        summary = strict_json((path / "stdout.json").read_text(encoding="utf-8"))
        dt = summary["legacy"]["config"]["dt"]
        for fraction in (.5, .9, 1.0):
            steps = arrival_thresholds(frames[0], summary["events"], fraction)
            for uid, group in groups.items():
                step = steps.get(int(uid))
                reason = None if step is not None else "not_reached_by_horizon" if group["command"] in ("gather", "route") else "not_applicable"
                interval_value = None if step is None else {"lower": max(0, step - 1) * dt, "upper": step * dt,
                                    "sparse": interval > 1, "source": "state_events", "resolution": "simulation_step"}
                percent = round(fraction * 100)
                group[f"time_to_{percent}_arrived"] = None if step is None else step * dt
                group[f"time_to_{percent}_reason"] = reason
                group[f"time_to_{percent}_interval"] = interval_value
        result = {"analysis_version": ANALYSIS_VERSION, "analysis_settings": meta["analysis_settings"], "groups": groups,
                  "agent_contacts": sum(e["type"] == "agent_contact" for e in summary["events"]),
                  "obstacle_contacts": sum(e["type"] == "obstacle_contact" for e in summary["events"]),
                  "first_contact_time": min((e["time"] for e in summary["events"] if e["type"] in ("agent_contact", "obstacle_contact")), default=None)}
        meta["analysis_seconds"] = time.perf_counter() - start
        phase = "plotting"
        plot_start = time.perf_counter()
        try:
            figures(frames, plot_rows, path / "plots")
        finally:
            meta["plotting_seconds"] = time.perf_counter() - plot_start
        (path / "metrics.csv.tmp").replace(path / "metrics.csv")
        write_json(path / "summary.json", result)
        write_json(path / "validation.json", {"valid": True, "frames": count, "plot_frames": len(frames), "warnings": sorted(warnings)})
        if has_metadata and meta:
            meta.update(status="success", analysis_version=ANALYSIS_VERSION)
            meta.pop("error", None)
            write_json(meta_path, meta)
    except (Exception, KeyboardInterrupt) as exc:
        status = "interrupted" if isinstance(exc, KeyboardInterrupt) else "invalid_output" if phase == "validation" else "analysis_failed"
        write_json(path / "validation.json", {"valid": False, "status": status, "error": str(exc) or status})
        (path / "metrics.csv.tmp").unlink(missing_ok=True)
        if has_metadata and meta:
            meta.update(status=status, error=str(exc) or status, analysis_version=ANALYSIS_VERSION)
            write_json(meta_path, meta)
        raise
