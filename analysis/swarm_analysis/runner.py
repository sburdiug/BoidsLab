import csv
import hashlib
import json
import platform
import subprocess
import sys
import statistics
import matplotlib.pyplot as plt
import time
from datetime import datetime, timezone
from pathlib import Path
import numpy
import matplotlib
from .experiments import load_manifest
from .reader import read_run
from .metrics import frame_metrics
from .plots import figures


def digest(data): return hashlib.sha256(data).hexdigest()
def write_json(path, data):
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")
    temp.replace(path)


def analyze(path, expected_steps=None):
    path=Path(path)
    try:
        frames, summary=read_run(path)
        if expected_steps is not None and summary["steps"] != expected_steps:
            raise ValueError(f"expected {expected_steps} steps, got {summary['steps']}")
        rows=[row for frame in frames for row in frame_metrics(frame)]
        with (path/"metrics.csv").open("w", encoding="utf-8", newline="") as f:
            if rows:
                writer=csv.DictWriter(f,fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
        by_group={}
        for group in summary["groups"]:
            data=[r for r in rows if r["group_uid"]==group["uid"]]
            arrived=next((r["time"] for r in data if group["command"]["type"] in ("gather","route") and r["arrived"] is not None and r["arrived"]>=.9),None)
            by_group[str(group["uid"]) ]={"final":data[-1],"time_to_90_arrived":arrived,"time_to_90_reason":None if arrived is not None else "not_reached_by_horizon" if group["command"]["type"] in ("gather","route") else "not_applicable", "max_terminated":max(r["terminated"] for r in data)}
        write_json(path/"summary.json",{"groups":by_group,"agent_contacts":sum(e["type"]=="agent_contact" for e in summary["events"]),"obstacle_contacts":sum(e["type"]=="obstacle_contact" for e in summary["events"])})
        figures(frames,rows,path/"plots")
        write_json(path/"validation.json",{"valid":True,"frames":len(frames),"warnings":[]})
    except Exception as exc:
        write_json(path/"validation.json",{"valid":False,"error":str(exc)})
        raise


def report(path):
    path=Path(path); runs=[]
    for meta in sorted(path.glob("*/metadata.json")):
        m=json.loads(meta.read_text(encoding="utf-8")); m["directory"]=meta.parent.name; runs.append(m)
    lines=["# Experiment report", "", "Exploratory results; agents are simplified and are not calibrated quadcopters.", "", f"Runs: {len(runs)}; successful: {sum(r['status']=='success' for r in runs)}; failed: {sum(r['status']!='success' for r in runs)}.", "", "| Case | Parameters | Status | Result |", "| --- | --- | --- | --- |"]
    for r in runs:
        link=f"[{r['directory']}]({r['directory']}/plots/dynamics.png)" if r['status']=='success' else r.get('error','')
        lines.append(f"| {r['case']} | {json.dumps(r['params'])} | {r['status']} | {link} |")
    for r in runs:
        if r['status']=='success':
            lines.extend(["",f"## {r['case']} Р Р†Р вЂљРІР‚Сњ {r['params']}","",f"![Trajectories]({r['directory']}/plots/trajectory.png)","",f"![Dynamics]({r['directory']}/plots/dynamics.png)"])
    lines += ["", "Arrived does not alone prove task success. Sampled distances do not prove collision freedom. Process timing includes startup and serialization."]
    combinations = {}
    for r in runs:
        key = (r["case"], json.dumps({k:v for k,v in r["params"].items() if k != "seed"}, sort_keys=True))
        combinations.setdefault(key, []).append(r)
    aggregated=[]
    for (case, params), items in combinations.items():
        values=[]
        for item in items:
            summary_path=path/item["directory"]/"summary.json"
            if item["status"] == "success" and summary_path.exists():
                result=json.loads(summary_path.read_text(encoding="utf-8"))
                for uid, group in result["groups"].items():
                    values.append({"seed":item["params"].get("seed"),"group_uid":uid,"polarization":group["final"]["polarization"],"terminated":group["final"]["terminated"],"time_to_90_arrived":group["time_to_90_arrived"]})
        for uid in sorted({v["group_uid"] for v in values}):
            group_values=[v for v in values if v["group_uid"] == uid]
            pol=[v["polarization"] for v in group_values if v["polarization"] is not None]
            arrived=[v["time_to_90_arrived"] for v in group_values if v["time_to_90_arrived"] is not None]
            aggregated.append({"case":case,"group_uid":uid,"parameters":json.loads(params),"planned":len(items),"successful":sum(x["status"]=="success" for x in items),"failed":sum(x["status"]!="success" for x in items),"polarization_mean":statistics.mean(pol) if pol else None,"polarization_sd":statistics.stdev(pol) if len(pol)>1 else None,"time_to_90_completion_rate":len(arrived)/len(group_values) if group_values else None,"seed_values":group_values})
    write_json(path/"aggregate.json",aggregated)
    lines.extend(["", "## Seed-level summaries", "", "Values are exploratory; SD describes seed variability. See aggregate.json for individual values."])
    for item in aggregated:
        lines.append(f"- {item['case']} {item['parameters']}: {item['successful']}/{item['planned']} successful; final polarization mean={item['polarization_mean']}, sample SD={item['polarization_sd']}; seeds={item['seed_values']}")
    alignment=[a for a in aggregated if a["case"]=="free_boids" and "alignment" in a["parameters"]]
    if alignment:
        fig,ax=plt.subplots(figsize=(7,4))
        for item in alignment:
            x=item["parameters"]["alignment"]
            for value in item["seed_values"]:
                if value["polarization"] is not None: ax.scatter(x,value["polarization"],color="steelblue",alpha=.6)
            if item["polarization_mean"] is not None:
                ax.errorbar(x,item["polarization_mean"],yerr=item["polarization_sd"] or 0,fmt="o",color="black",capsize=4)
        ax.set(xlabel="Alignment weight",ylabel="Final polarization",title="Seed values, mean and sample SD"); ax.grid(alpha=.2); fig.tight_layout(); fig.savefig(path/"alignment.png",dpi=140); plt.close(fig)
        lines.extend(["", "![Alignment sweep](alignment.png)"])
    (path/"report.md").write_text("\n".join(lines)+"\n",encoding="utf-8")


def run(manifest, binary, output, timeout):
    manifest=Path(manifest).resolve(); binary=Path(binary).resolve(); output=Path(output).resolve()
    cases=list(load_manifest(manifest)); binary_hash=digest(binary.read_bytes())
    output.mkdir(parents=True,exist_ok=False)
    print(f"Planned {len(cases)} trials",flush=True)
    failed=False
    root=Path(__file__).resolve().parents[2]
    try:
        revision=subprocess.run(["git","rev-parse","HEAD"],cwd=root,capture_output=True,text=True,check=True).stdout.strip()
        dirty=bool(subprocess.run(["git","status","--porcelain"],cwd=root,capture_output=True,text=True,check=True).stdout.strip())
    except (OSError, subprocess.CalledProcessError): revision=dirty=None
    for index, case in enumerate(cases,1):
        scene_bytes=case["scene"].read_bytes() if case["scene"] else b""
        ident=digest(json.dumps([case["args"],case["params"],binary_hash,digest(scene_bytes),1],sort_keys=True).encode())
        folder=output/(case["name"]+"-"+ident[:12]); folder.mkdir()
        args=[str(binary),*case["args"]]
        if case["scene"]:
            (folder/"scene.swarm").write_bytes(scene_bytes); args += ["--scene",str(folder/"scene.swarm")]
        args += ["--trace",str(folder/"trace.jsonl")]
        meta={"case":case["name"],"params":case["params"],"argv":args,"run_hash":ident,"binary_sha256":binary_hash,"scene_sha256":digest(scene_bytes) if case["scene"] else None,"git_commit":revision,"git_dirty":dirty,"binary_origin":"unknown", "python":sys.version,"platform":platform.platform(),"numpy":numpy.__version__,"matplotlib":matplotlib.__version__,"started_utc":datetime.now(timezone.utc).isoformat()}
        start=time.monotonic()
        try:
            with (folder/"stdout.json").open("wb") as stdout, (folder/"stderr.txt").open("wb") as stderr:
                result=subprocess.run(args,stdout=stdout,stderr=stderr,timeout=timeout,check=False)
            meta["exit_code"]=result.returncode
            if result.returncode: meta["status"]="process_failed"; meta["error"]=(folder/"stderr.txt").read_text(encoding="utf-8",errors="replace")[:500]
            else:
                try:
                    expected_steps = int(case["args"][case["args"].index("--steps") + 1]) if "--steps" in case["args"] else 100
                    analyze(folder, expected_steps); meta["status"]="success"
                except Exception as exc: meta["status"]="invalid_output"; meta["error"]=str(exc)
        except subprocess.TimeoutExpired: meta["status"]="timeout"; meta["error"]="process exceeded timeout"
        except OSError as exc: meta["status"]="process_failed"; meta["error"]=str(exc)
        meta["elapsed_seconds"]=time.monotonic()-start; meta["finished_utc"]=datetime.now(timezone.utc).isoformat(); write_json(folder/"metadata.json",meta)
        if meta["status"] != "success": failed=True
        print(f"{index}/{len(cases)} {case['name']} {case['params']}: {meta['status']}",flush=True)
    report(output)
    return 1 if failed else 0
