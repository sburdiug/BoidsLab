"""Validated static-run streaming reader. A final summary is held once, never all frames."""
import json
import math
from pathlib import Path


class InvalidOutput(ValueError):
    pass


def strict_json(text):
    def check(value):
        if isinstance(value, (int, float)):
            try:
                if not math.isfinite(value): raise InvalidOutput("non-finite number")
            except OverflowError as exc:
                raise InvalidOutput("number outside supported range") from exc
        if isinstance(value, dict):
            for item in value.values(): check(item)
        elif isinstance(value, list):
            for item in value: check(item)
    value = json.loads(text)
    check(value)
    return value


def option(argv, flag, default):
    indices = [i for i, value in enumerate(argv) if value == flag]
    return int(argv[indices[-1] + 1]) if indices else default


def natural(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def iter_run(directory, expected_steps=None, trace_every=None):
    directory = Path(directory)
    meta_path = directory / "metadata.json"
    if meta_path.exists():
        meta = strict_json(meta_path.read_text(encoding="utf-8"))
        if meta.get("exit_code") != 0 or meta.get("process_status", meta.get("status")) in ("timeout", "process_failed", "interrupted", "setup_failed"):
            raise InvalidOutput("process did not finish successfully")
        expected_steps = option(meta["argv"], "--steps", 100) if expected_steps is None else expected_steps
        trace_every = option(meta["argv"], "--trace-every", 1) if trace_every is None else trace_every
    summary = strict_json((directory / "stdout.json").read_text(encoding="utf-8"))
    horizon = summary["steps"] if expected_steps is None else expected_steps
    interval = 1 if trace_every is None else trace_every
    if interval < 1 or horizon < 0 or summary["steps"] != horizon or summary["completion_reason"] != "horizon" or summary["event_scope"] != "full_history":
        raise InvalidOutput("invalid horizon or event scope")
    previous = None
    signature = None
    sequence = 0
    last_event_step = 0
    states = None
    expected = 0
    with (directory / "trace.jsonl").open(encoding="utf-8") as source:
        for line_no, line in enumerate(source, 1):
            try:
                if not line.endswith("\n"): raise InvalidOutput("incomplete line")
                frame = strict_json(line)
                if not natural(frame["steps"]): raise InvalidOutput("invalid step")
                if frame["model"] not in ("boids-2d-v13", "boids-2d-v14") or frame["schema_version"] != 2 or frame["units"] != {"length":"m", "time":"s", "speed":"m/s"}:
                    raise InvalidOutput("unsupported model/units")
                if expected is None or frame["steps"] != expected or frame["event_scope"] != "incremental":
                    raise InvalidOutput("unexpected frame sequence")
                expected = None if expected == horizon else min(expected + interval, horizon)
                groups = {g["uid"]: g for g in frame["groups"]}
                agents = frame["agents"]
                if states is None:
                    states = {a["uid"]: a["state"] for a in agents}
                agent_uids = {a["uid"] for a in agents}
                if any(not natural(uid) for uid in [*groups, *agent_uids]): raise InvalidOutput("invalid UID")
                if len(groups) != len(frame["groups"]) or len(agent_uids) != len(agents): raise InvalidOutput("duplicate UID")
                current = (frame["model"], frame["groups"], frame["scene"], frame["paths"], [(a["uid"],a["group_uid"]) for a in agents])
                if signature is not None and current != signature: raise InvalidOutput("dynamic scenes/groups unsupported")
                signature = current
                dt = frame["legacy"]["config"]["dt"]
                if not isinstance(dt,(int,float)) or isinstance(dt,bool) or dt <= 0: raise InvalidOutput("invalid dt")
                if not math.isclose(frame["time"],frame["steps"]*dt,abs_tol=1e-9): raise InvalidOutput("invalid frame time")
                for g in groups.values():
                    c = g["config"]
                    for k in ("dt","speed","neighbor_radius","separation_radius","body_radius"):
                        if not isinstance(c[k],(int,float)) or isinstance(c[k],bool) or c[k] <= 0: raise InvalidOutput("invalid " + k)
                    if c["dt"] != dt or c["separation_radius"] > c["neighbor_radius"] or isinstance(c["turn_response"],bool) or not 0 <= c["turn_response"] <= 1: raise InvalidOutput("invalid group parameters")
                    for k in ("separation_weight","alignment_weight","cohesion_weight","command_weight","obstacle_clearance","obstacle_weight","route_centering","route_cohesion"):
                        if isinstance(c[k],bool) or not isinstance(c[k],(int,float)) or c[k] < 0: raise InvalidOutput("invalid " + k)
                for a in agents:
                    if a["group_uid"] not in groups or a["group_id"] != groups[a["group_uid"]]["id"] or a["state"] not in ("moving","arrived","terminated"): raise InvalidOutput("invalid agent reference/state")
                    for k in ("position","velocity","measured_position","air_velocity","wind_velocity"):
                        if len(a[k]) != 2 or any(isinstance(x,bool) or not isinstance(x,(int,float)) for x in a[k]): raise InvalidOutput("invalid " + k)
                if frame["initial_agents"] != len(agents): raise InvalidOutput("population mismatch")
                for state in ("moving","arrived","terminated"):
                    if frame[state] != sum(a["state"]==state for a in agents): raise InvalidOutput("state count mismatch")
                for event in frame["events"]:
                    if not natural(event["sequence"]) or not natural(event["step"]): raise InvalidOutput("invalid event index")
                    if any(not natural(uid) for uid in event["agent_uids"] + event["group_uids"]): raise InvalidOutput("invalid event UID")
                    if event["sequence"] != sequence or not 0 <= event["step"] <= frame["steps"]: raise InvalidOutput("invalid event sequence/step")
                    if event["step"] < last_event_step: raise InvalidOutput("events out of step order")
                    last_event_step = event["step"]
                    if not 0 <= event["time"] <= frame["time"]+1e-9 or not (event["step"]-1)*dt-1e-9 <= event["time"] <= event["step"]*dt+1e-9: raise InvalidOutput("invalid event time")
                    if set(event["agent_uids"])-agent_uids or set(event["group_uids"])-groups.keys(): raise InvalidOutput("unknown event UID")
                    if event["step"] > 0 and event["type"] not in ("state_changed","agent_contact","obstacle_contact"): raise InvalidOutput("dynamic event unsupported")
                    if event["step"] > 0 and event["type"] == "state_changed":
                        if len(event["agent_uids"]) != 1 or event["to"] not in ("moving", "arrived", "terminated"):
                            raise InvalidOutput("invalid state event")
                        uid = event["agent_uids"][0]
                        if states[uid] != event["from"]: raise InvalidOutput("state event history mismatch")
                        states[uid] = event["to"]
                    if sequence >= len(summary["events"]) or event != summary["events"][sequence]: raise InvalidOutput("event history mismatch")
                    sequence += 1
                if states != {a["uid"]: a["state"] for a in agents}: raise InvalidOutput("state events differ from snapshot")
                previous = frame
                yield frame
            except (KeyError, TypeError, ValueError, IndexError) as exc:
                raise InvalidOutput(f"trace.jsonl line {line_no}: {exc}") from exc
    if previous is None or expected is not None: raise InvalidOutput("missing final frame")
    if sequence != len(summary["events"]): raise InvalidOutput("missing events")
    for key in ("model","schema_version","units","steps","time","groups","agents","paths","scene","moving","arrived","terminated"):
        if summary[key] != previous[key]: raise InvalidOutput("final snapshot mismatch: " + key)


def read_run(directory):
    # Compatibility for small control tests; production analysis uses iter_run.
    frames = list(iter_run(directory))
    return frames, strict_json((Path(directory)/"stdout.json").read_text(encoding="utf-8"))
