# Swarm analysis

Runs the Rust simulator and produces per-frame CSV metrics, per-run plots, seed summaries and a Markdown report. Model physics remain in Rust. Requires Python 3.11+, NumPy and Matplotlib. Tested with Python 3.12.10, NumPy 2.5.3, Matplotlib 3.11.1 on Windows.

From the repository root in PowerShell:

```powershell
cargo build --release --locked --bin simulation
cd analysis
python -m pip install -e .
python -m unittest discover -s tests
python -m swarm_analysis run experiments/smoke.json --binary ../target/release/simulation.exe --output ../work/analysis/smoke --timeout 60
python -m swarm_analysis run experiments/first_experiments.json --binary ../target/release/simulation.exe --output ../work/analysis/first --timeout 60
```

Choose a new output directory on each run: existing folders are not overwritten. The Rust executable must be built before running; the tool never rebuilds it automatically. On other operating systems use the appropriate executable name. Open `report.md` from the suite folder. Each run keeps `metadata.json`, raw `stdout.json`, `stderr.txt`, `trace.jsonl`, validation, metrics CSV, summary and plots. `aggregate.json` holds per-seed values and summary statistics. A preset can be included in a manifest as a `scene` path relative to the manifest; its contents are copied before running.

To recompute derived files from a run, or rebuild a suite report:

```powershell
python -m swarm_analysis analyze ../work/analysis/first/RUN_DIRECTORY
python -m swarm_analysis report ../work/analysis/first
```

Experiment descriptions use JSON. See `experiments/first_experiments.json`: each case lists Rust CLI tokens and a grid. `${name}` is substituted only as a whole token; grids expand in sorted key order. `--scene`, `--trace` and `--trace-every` are managed by the runner. `--group` parameters belong to that group; specify profiles explicitly to avoid inheritance. Successful and failed runs are listed separately; a suite with failures exits nonzero.

## Interpretation and current limits

- `arrived` includes Hold and is not by itself evidence of successful navigation. Target-circle membership is descriptive; the simulated region is soft.
- A contact event is not necessarily a unique collision incident when contacts are simultaneous. Body gaps from snapshots cannot guarantee absence of swept collisions.
- The analysis supports schema 2 / `boids-2d-v13`, fixed groups and static commands. It records every simulation step. UIDs are scoped to a single run.
- The first version loads a whole trace in memory and computes pair distances with quadratic memory (pair calculations are skipped beyond 2 million pairs). It is intended for the included small experiments, not large-scale workloads. Skipped pair metrics appear blank.
- Cross-platform bitwise reproducibility is not promised. A supplied binary's source revision cannot be verified from its hash; its origin is marked unknown.
- Current reports aggregate final polarization by seed. They do not calculate uncertainty intervals or select an optimal setting. A process wall time includes initialization and output; it is not GUI FPS or core-only timing.
