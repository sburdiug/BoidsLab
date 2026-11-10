import json
import math
from pathlib import Path


def strict_json(s):
    return json.loads(s, parse_constant=lambda x: (_ for _ in ()).throw(ValueError("non-finite number: " + x)))


def read_run(directory):
    directory = Path(directory)
    summary = strict_json((directory / "stdout.json").read_text(encoding="utf-8"))
    frames, events, signature = [], [], None
    with (directory / "trace.jsonl").open(encoding="utf-8") as source:
        for line_no, line in enumerate(source, 1):
            try:
                if not line.endswith("\n"):
                    raise ValueError("incomplete line")
                f = strict_json(line)
                if f["model"] != "boids-2d-v13" or f["schema_version"] != 2 or f["units"] != {"length": "m", "time": "s", "speed": "m/s"}:
                    raise ValueError("unsupported model or units")
                if f["event_scope"] != "incremental" or f["steps"] != len(frames):
                    raise ValueError("unexpected event scope or step sequence")
                groups = {g["uid"]: g for g in f["groups"]}
                agents = f["agents"]
                if len(groups) != len(f["groups"]) or len({a["uid"] for a in agents}) != len(agents):
                    raise ValueError("duplicate UID")
                current = (f["groups"], f["scene"], [(a["uid"], a["group_uid"]) for a in agents])
                if signature is None:
                    signature = current
                elif current != signature:
                    raise ValueError("dynamic groups, commands or scenes are unsupported")
                if any(a["group_uid"] not in groups or a["state"] not in ("moving", "arrived", "terminated") for a in agents):
                    raise ValueError("invalid agent group or state")
                for a in agents:
                    for key in ("position", "velocity", "measured_position", "air_velocity", "wind_velocity"):
                        if len(a[key]) != 2 or any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in a[key]):
                            raise ValueError("invalid " + key)
                for state in ("moving", "arrived", "terminated"):
                    if f[state] != sum(a["state"] == state for a in agents):
                        raise ValueError("state count mismatch")
                if f["initial_agents"] != len(agents):
                    raise ValueError("population mismatch")
                for g in groups.values():
                    if not math.isclose(f["time"], f["steps"] * g["config"]["dt"], abs_tol=1e-9):
                        raise ValueError("time mismatch")
                for event in f["events"]:
                    if event["sequence"] != len(events) or event["step"] > f["steps"]:
                        raise ValueError("invalid event sequence")
                    events.append(event)
                frames.append(f)
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"trace.jsonl line {line_no}: {exc}") from exc
    if not frames:
        raise ValueError("empty trace")
    if summary["event_scope"] != "full_history" or summary["events"] != events:
        raise ValueError("summary event history differs from trace")
    last = frames[-1]
    for key in ("model", "schema_version", "units", "steps", "time", "groups", "agents", "paths", "scene", "moving", "arrived", "terminated"):
        if summary[key] != last[key]:
            raise ValueError("final snapshot mismatch: " + key)
    return frames, summary
