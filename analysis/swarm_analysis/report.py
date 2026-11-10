"""Aggregate valid observations without hiding failed or inapplicable trials."""
import json
import statistics
from pathlib import Path
from .analyzer import write_json
from .plots import alignment_figure


def report(path):
    path = Path(path)
    runs, combinations = [], {}
    for source in sorted(path.glob("*/metadata.json")):
        run = json.loads(source.read_text(encoding="utf-8"))
        run["directory"] = source.parent.name
        validation_path = source.parent / "validation.json"
        validation = json.loads(validation_path.read_text()) if validation_path.exists() else {}
        run["valid"] = run["status"] == "success" and validation.get("valid") is True and (source.parent / "summary.json").exists()
        run["warnings"] = validation.get("warnings", []) if run["valid"] else []
        runs.append(run)
        key = (run["case"], json.dumps({k: v for k, v in run["params"].items() if k != "seed"}, sort_keys=True))
        combinations.setdefault(key, []).append(run)
    aggregate = []
    for (case, params), items in combinations.items():
        values = []
        for item in items:
            if not item["valid"]:
                continue
            result = json.loads((path / item["directory"] / "summary.json").read_text())
            for uid, group in result["groups"].items():
                values.append({"seed": item["params"].get("seed"), "group_uid": uid,
                    "polarization": group["final"]["polarization"], "terminated": group["final"]["terminated"],
                    "spread": group["final"]["spread"], "time_to_90_arrived": group["time_to_90_arrived"],
                    "time_to_90_reason": group["time_to_90_reason"], "time_to_90_interval": group.get("time_to_90_interval"),
                    "minimum_sampled_world_body_gap": group.get("minimum_sampled_world_body_gap"),
                    "world_agent_contacts": result["agent_contacts"], "world_obstacle_contacts": result["obstacle_contacts"]})
        for uid in sorted({v["group_uid"] for v in values}) or [None]:
            samples = [v for v in values if v["group_uid"] == uid]
            applicable = [v for v in samples if v["time_to_90_reason"] != "not_applicable"]
            completed = sum(v["time_to_90_arrived"] is not None for v in applicable)
            item = {"case": case, "group_uid": uid, "parameters": json.loads(params), "planned": len(items),
                "valid": sum(i["valid"] for i in items), "failed": sum(not i["valid"] for i in items),
                "applicable_valid": len(applicable), "completed": completed,
                "conditional_completion_rate": completed / len(applicable) if applicable else None,
                "completion_reason": None if applicable else "not_applicable" if samples else "no_group_observations",
                "seed_values": samples, "warnings": sorted({w for i in items for w in i["warnings"]})}
            for metric in ("polarization", "terminated", "spread", "minimum_sampled_world_body_gap", "time_to_90_arrived"):
                observed = [v[metric] for v in samples if v[metric] is not None]
                item[metric + "_mean"] = statistics.mean(observed) if observed else None
                item[metric + "_sd"] = statistics.stdev(observed) if len(observed) > 1 else None
            aggregate.append(item)
    write_json(path / "aggregate.json", aggregate)
    def cell(value):
        return str(value).replace("|", "\\|").replace("\n", " ")
    def number(value):
        return "n/a" if value is None else f"{value:.4g}"
    lines = ["# Experiment report", "", "Exploratory simplified-agent results; no calibrated vehicle model or optimal configuration is claimed.", "",
        f"Runs: {len(runs)}; valid: {sum(r['valid'] for r in runs)}; failed/incomplete: {sum(not r['valid'] for r in runs)}.", "",
        "| Case | Parameters | Status | Result |", "| --- | --- | --- | --- |"]
    for run in runs:
        link = f"[Plots]({run['directory']}/plots/dynamics.png)" if run["valid"] else run.get("error", "No validated result")
        lines.append("| " + " | ".join(cell(v) for v in (run["case"], json.dumps(run["params"]), run["status"], link)) + " |")
    lines += ["", "## Safety, task and cohesion across seeds", "",
        "Completion is conditional on valid, applicable group observations. Arrival time averages include completed trials only; compare with failures and completion counts. Spread and termination are final values. Gaps are sampled world minima, repeated per group.", "",
        "| Case / group | Parameters | Planned / valid / failed | Completed / applicable | Completion rate | Terminated mean | Spread mean (m) | Sampled world gap mean (m) | Arrival upper bound mean (s) |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for item in aggregate:
        rate = number(item["conditional_completion_rate"]) if item["conditional_completion_rate"] is not None else item["completion_reason"]
        vals = (f"{item['case']} / {item['group_uid']}", json.dumps(item["parameters"]), f"{item['planned']} / {item['valid']} / {item['failed']}",
            f"{item['completed']} / {item['applicable_valid']}", rate, number(item["terminated_mean"]), number(item["spread_mean"]),
            number(item["minimum_sampled_world_body_gap_mean"]), number(item["time_to_90_arrived_mean"]))
        lines.append("| " + " | ".join(cell(v) for v in vals) + " |")
    lines += ["", "Individual seed values, polarization, sample SD and undefined reasons are in aggregate.json and each run's summary/CSV. No confidence intervals are inferred."]
    if alignment_figure(aggregate, path / "alignment.png"):
        lines += ["", "![Alignment sweep](alignment.png)"]
    for run in runs:
        if run["valid"]:
            lines += ["", f"## {run['case']} — {run['params']}", ""]
            lines.extend(f"- Warning: {warning}" for warning in run["warnings"])
            lines += ["", f"![Trajectories]({run['directory']}/plots/trajectory.png)", "", f"![Dynamics]({run['directory']}/plots/dynamics.png)"]
    lines += ["", "Arrived includes Hold and does not alone prove task success. Sampled distances cannot prove collision freedom. Rust process time includes startup and serialization; analysis and plotting are timed separately."]
    (path / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
