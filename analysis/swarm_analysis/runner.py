import hashlib
import json
import math
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
import numpy
import matplotlib
from .experiments import load_manifest
from .analyzer import analyze, write_json, ANALYSIS_VERSION
from .report import report


def digest(data):
    return hashlib.sha256(data).hexdigest()


def run(manifest, binary, output, timeout, trace_every=1, component_radius=1.0):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be positive and finite")
    if isinstance(trace_every, bool) or not isinstance(trace_every, int) or trace_every < 1:
        raise ValueError("trace_every must be a positive integer")
    if isinstance(component_radius, bool) or not isinstance(component_radius, (int, float)) or not math.isfinite(component_radius) or component_radius <= 0:
        raise ValueError("component radius must be positive and finite")
    manifest, binary, output = (Path(p).resolve() for p in (manifest, binary, output))
    cases = list(load_manifest(manifest))
    binary_hash = digest(binary.read_bytes())
    output.mkdir(parents=True, exist_ok=False)
    (output / "manifest.json").write_bytes(manifest.read_bytes())
    print(f"Planned {len(cases)} trials", flush=True)
    root = Path(__file__).resolve().parents[2]
    try:
        revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, check=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain"], cwd=root, capture_output=True, text=True, check=True).stdout.strip())
    except (OSError, subprocess.CalledProcessError):
        revision = dirty = None
    for index, case in enumerate(cases, 1):
        folder = output / f"{index:04d}-{case['name']}"
        folder.mkdir()
        write_json(folder / "metadata.json", {"case": case["name"], "params": case["params"], "status": "not_started", "analysis_settings": {"component_radius_m": component_radius}})
    failed = False
    for index, case in enumerate(cases, 1):
        folder = output / f"{index:04d}-{case['name']}"
        args = [str(binary), *case["args"]]
        meta = {"case": case["name"], "params": case["params"], "argv": args, "binary_sha256": binary_hash,
            "git_commit": revision, "git_dirty": dirty, "binary_origin": "unknown", "analysis_version": ANALYSIS_VERSION,
            "analysis_settings": {"component_radius_m": component_radius},
            "python": sys.version, "platform": platform.platform(), "numpy": numpy.__version__, "matplotlib": matplotlib.__version__,
            "started_utc": datetime.now(timezone.utc).isoformat(), "status": "preparing", "exit_code": None}
        write_json(folder / "metadata.json", meta)
        phase = "setup"
        try:
            scene_bytes = case["scene"].read_bytes() if case["scene"] else b""
            meta["scene_sha256"] = digest(scene_bytes) if case["scene"] else None
            meta["run_hash"] = digest(json.dumps([case["name"], case["args"], case["params"], binary_hash, digest(scene_bytes), trace_every, ANALYSIS_VERSION], sort_keys=True).encode())
            if case["scene"]:
                (folder / "scene.swarm").write_bytes(scene_bytes)
                args += ["--scene", str(folder / "scene.swarm")]
            args += ["--trace", str(folder / "trace.jsonl"), "--trace-every", str(trace_every)]
            meta["argv"] = args
            meta["status"] = "running"
            write_json(folder / "metadata.json", meta)
            phase = "process"
            start = time.perf_counter()
            try:
                with (folder / "stdout.json").open("wb") as stdout, (folder / "stderr.txt").open("wb") as stderr:
                    with subprocess.Popen(args, stdout=stdout, stderr=stderr) as process:
                        try:
                            meta["exit_code"] = process.wait(timeout=timeout)
                        except (subprocess.TimeoutExpired, KeyboardInterrupt):
                            process.kill()
                            process.wait()
                            raise
                meta["status"] = "process_complete" if meta["exit_code"] == 0 else "process_failed"
                if meta["exit_code"]:
                    meta["error"] = (folder / "stderr.txt").read_text(encoding="utf-8", errors="replace")[:500]
            finally:
                meta["process_seconds"] = time.perf_counter() - start
            meta["process_status"] = meta["status"]
            write_json(folder / "metadata.json", meta)
            if meta["exit_code"] == 0:
                phase = "analysis"
                analyze(folder)
                meta = json.loads((folder / "metadata.json").read_text())
        except (Exception, KeyboardInterrupt) as exc:
            if phase == "analysis":
                meta = json.loads((folder / "metadata.json").read_text())
            else:
                meta["status"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "timeout" if isinstance(exc, subprocess.TimeoutExpired) else "setup_failed" if phase == "setup" else "process_failed"
                meta["process_status"] = meta["status"]
                meta["error"] = str(exc) or meta["status"]
            if isinstance(exc, KeyboardInterrupt):
                write_json(folder / "metadata.json", meta)
                report(output)
                return 130
        meta["finished_utc"] = datetime.now(timezone.utc).isoformat()
        write_json(folder / "metadata.json", meta)
        failed |= meta["status"] != "success"
        print(f"{index}/{len(cases)} {case['name']} {case['params']}: {meta['status']}", flush=True)
    report(output)
    return int(failed)
