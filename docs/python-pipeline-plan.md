# Python analysis pipeline — implementation handoff

Status: design, not implemented. **Update 2026-11-10:** the export is now `boids-2d-v13` / schema 2; use stable `uid`, contact events, `event_scope`, and optional `--trace-every`. See [current export notes](playground.md#export-format-v13). The v12-specific statements below are historical and must be adapted before implementation. Reviewed 2026-11-10 against Rust commit `b7894dbe7153f08d58f2d2b3d6721e76e22f6426`.

Implement the first useful analysis workflow: a few verified runs, reproducible experiments, CSV metrics and readable plots. Rust remains the only simulation engine. Do not rewrite its movement rules in Python. Read [the architecture review](architecture-review-2026-11-10.md) before implementation. The [Ukrainian overview](python-pipeline-plan.uk.md) summarizes the decisions; this English document is the implementation contract.

## 1. Scope and deliverables

Deliver `analysis/` with an installable Python package, a command-line entry point, small checked-in experiment descriptions, tests, and an English README with Windows commands. Keep generated runs under ignored `work/analysis/`. Use Python 3.11 or later, standard-library JSON/subprocess/csv/pathlib/argparse, NumPy and Matplotlib; pytest is the test dependency. Record the tested dependency versions. Do not introduce pandas, SciPy, a database, notebooks as the execution engine, dashboards, optimizers, Rust bindings or concurrent workers for this first version.

The completed workflow is:

`experiment JSON -> explicit CLI arguments -> Rust process -> raw files -> validation -> per-frame metrics -> per-run summary -> seed aggregation -> PNG plots + Markdown report`

One command runs a suite; another re-analyzes an existing run without re-running Rust. No automatic search for an "ideal" configuration. A result includes failures and unfinished trials, not just successful examples.

## 2. Read these sources first

Paths are relative to the repository root:

| Source | What to verify |
| --- | --- |
| `src/bin/simulation.rs` | Run initialization, trace frame ordering, exit behavior |
| `src/bin/simulation/options.rs` | Flags, group ordering, inheritance, validation |
| `src/bin/simulation/output.rs` | Actual JSON field names and geometry conventions |
| `src/simulation/config.rs` | Default values and valid ranges |
| `src/simulation/{agent,commands,collision,events}.rs` | Velocity, arrival and event semantics |
| `src/simulation/{geometry,scene_preset,groups}.rs` | Rectangles, saved environment, index changes |
| `tests/{cli,presets,gather,groups}.rs` | Existing behavior checks |

Re-check these files if HEAD differs. Document any changed contract instead of silently guessing. The executable does not expose build provenance, so Python must record it itself.

## 3. Rust/Python data contract

- Build once using `cargo build --release --locked --bin simulation`; do not invoke Cargo for every trial. Use `--offline` only when dependencies are available. Build failure stops the suite.
- Run the resulting executable with an argument list and `shell=False`. Support a user-supplied executable path. Resolve repository, manifest, scene and executable paths explicitly; support spaces and Unicode paths on Windows.
- Stdout is one JSON summary, stderr is diagnostics. `--trace PATH` writes JSONL: initial step 0 plus every completed step. For `--steps S`, expect exactly `S+1` frames, including when S=0. Step 0 has reason `snapshot`; the summary always has reason `horizon`. Do not require a step-0 trace to say `horizon`.
- Supported model is initially exactly `boids-2d-v12`. Validate units `m`, `s`, `m/s`. Model name is not sufficient provenance: behavior has changed without changing that name.
- Use `groups[*].config`, `spawn`, and `command` for group-specific values. Top-level config/command describe the first group; top-level seed/spawn_radius are legacy CLI values and can disagree with explicit groups.
- `agents[*].position` is true world position; `measured_position` is noisy observation. `velocity` is ground velocity; `air_velocity` excludes wind. Physical metrics use true positions and ground velocities by default.
- States are lowercase `moving`, `arrived`, `terminated`. Terminated agents remain in snapshots and must be excluded from living-flock metrics. Hold produces arrived agents without completing a travel task.
- Trace events are incremental and have `sequence`, `step`, `time`. Summary events repeat the full history. Analyze trace events once; do not concatenate summary events. State changes caused directly by commands are not all separate state_changed events: snapshots are authoritative for counts.
- Termination events do not contain collision partners, obstacle IDs or exact substep contact time. Count terminated agents, not collision incidents; do not infer agent-agent incidents by dividing by two.
- Rectangle JSON uses `width` along local X and `height` along local Y, rotated counterclockwise by `angle_degrees`. Wind/noise JSON uses `length` along local X and `width` along local Y. Normalize this once in the reader. Do not use `scene.rectangle` in addition to `scene.rectangles`: it duplicates the first rectangle.
- A route's `radius` is half its `width`; geometry is a wide corridor around a polyline. A saved path in a preset is not automatically assigned to any group. First-version route experiments must supply explicit `--waypoint` arguments.
- No timed commands, runtime group deletion or live trace recording from the viewer are exposed by the current CLI. First-version analysis accepts static group membership and commands after initialization. Reject traces with later group/profile/command/scene changes with a clear unsupported-feature error. Group/agent indices shift on deletion, so treating them as permanent IDs would be wrong.
- Presets contain environment only. Copy the preset into the run directory and pass that copy to `--scene`; preserve its bytes and hash. Do not add a second Python preset parser for the MVP. Read plotted geometry from the Rust output.

## 4. Experiment description and CLI construction

Use one small JSON schema, with `schema_version: 1`, `name`, and `cases`. Each case has a unique `name`, a `grid` of named numeric parameters, an `args` list of CLI tokens, and optional `scene` path relative to the manifest. A placeholder must occupy an entire token, e.g. `${seed}`. Expand the Cartesian product in sorted parameter-name order; substitute finite numbers using locale-independent formatting. Reject missing variables, unused grid keys, empty grids, duplicate case names and unknown manifest keys. No expressions or executable templates.

Example (an actual valid starting manifest):

```json
{
  "schema_version": 1,
  "name": "first_alignment_sweep",
  "cases": [{
    "name": "free_boids",
    "grid": {"seed": [7, 42, 99], "alignment": [0, 1, 3]},
    "args": [
      "--steps", "200", "--dt", "0.1",
      "--group", "0", "0", "20", "3", "${seed}", "boids",
      "--speed", "1", "--turn-response", "0.1",
      "--neighbor-radius", "1", "--separation-radius", "0.3",
      "--body-radius", "0.025", "--separation-weight", "10",
      "--alignment-weight", "${alignment}", "--cohesion-weight", "3",
      "--command-weight", "15", "--obstacle-clearance", "0.15",
      "--obstacle-weight", "1", "--route-centering", "0.25",
      "--route-cohesion", "1"
    ]
  }]
}
```

Reserve `--trace` and `--scene` for the runner; reject them in args. Use explicit groups for all populated experiments; empty control cases use `--empty`. All 14 profile settings above must be explicit after each group (dt is global and appears before groups). This prevents accidental inheritance between groups. Group commands follow that group's options; directed `--interaction` flags may follow all groups. Validate the supplied manifests, while leaving Rust as the authority for model constraints. Invalid runtime parameters produce a failed run, not a Python traceback that loses the suite.

Expose only these initial commands:

```text
python -m swarm_analysis run experiments/smoke.json --binary PATH --output DIR --timeout 60
python -m swarm_analysis analyze RUN_DIR
python -m swarm_analysis report SUITE_DIR
```

`run` executes validation, analysis and report generation too. `analyze` and `report` allow rebuilding derived artifacts. All paths in printed instructions should be usable from `analysis/`. Use a unique new suite output directory; fail if it already exists. No automatic resume in v1. Reanalysis may replace derived files atomically, but never modify raw files.

## 5. Execution, provenance and failure handling

Run sequentially. Print case/progress/status. Give each expanded run a directory with a readable case name and short SHA-256 of canonical expanded args + preset bytes + executable hash + analysis schema version. Retain full hashes in metadata and detect directory collisions. Never rely on Python's built-in hash for identity.

Per-run files:

```text
metadata.json             # requested args, grid values, versions, hashes, timestamps, status
scene.swarm               # only if a preset was supplied
stdout.json               # raw output, may be incomplete on failure
stderr.txt
trace.jsonl               # raw, may be incomplete on failure
validation.json           # passed checks, errors, warnings
metrics.csv               # one row per frame and group
summary.json              # metrics plus applicability/undefined reasons
plots/trajectory.png
plots/dynamics.png
```

Use stdout/stderr file handles instead of holding subprocess output in RAM. Timeout must terminate and reap the process and close handles. Preserve partial artifacts. Record statuses `success`, `process_failed`, `timeout`, `invalid_output`, `analysis_failed`; record exit code and elapsed wall time separately. Continue independent cases after individual failure, return nonzero if any case failed, and still create a report of failures. Keyboard interruption terminates the current child, marks it interrupted, and stops the suite. Never mark a timed-out or truncated run successful.

Metadata includes UTC timestamps, git commit + dirty flag (or unavailable), executable SHA-256, preset SHA-256, Cargo.lock hash when available, exact argv, Python/platform/dependency versions, and analysis version. A supplied binary is not proven to come from the current checkout: record build origin as unknown unless the runner built it. Measure process wall time only; it includes initialization and serialization and is not core step timing or GUI FPS.

For first manifests use at most 50 agents, 300 steps, and 30 trials. Print the planned trial and agent-frame counts before starting. These are starter-manifest bounds, not new engine caps. Streaming analysis must not load the entire JSONL into a DataFrame or retain every frame for plots. Retain at most 1,000 trajectory samples per agent, chosen deterministically; use all frames for metrics. Future scale experiments need explicit trace-size planning.

## 6. Validation before trusting a run

Parse JSON with rejection of NaN/Infinity, not Python's permissive defaults. For each trace line check required fields/types, finite numeric vectors of length two, recognized states, supported model/units, unique agent/group IDs, valid group references, and fixed membership/config/commands after initialization. Require monotonically consecutive steps 0..S and `time ~= steps * dt`; all group dt values must agree. Check `moving + arrived + terminated == len(agents)` and the state-derived counts. In the supported static CLI, initial_agents must stay equal to the initial population.

Check event sequences for no duplicates/gaps; event step cannot exceed containing frame step. Accept initialization events at step 0. Compare the final trace and stdout summary for identical model state/config/scene/groups/paths, excluding `events` and `completion_reason`; compare concatenated trace events with summary events separately. Reject partial final lines, missing final frames and nonzero exit status. Report file + line + field for malformed data. Allow extra JSON keys within the supported model, but never invent values for missing required keys. Save failures to validation.json.

## 7. First metrics and their definitions

Compute per group, using original group count N0 as the denominator for fractions. Preserve run ID, case, seed/grid values, group ID, step and time in outputs. Do not pool two unrelated groups into one flock statistic.

| Metric | Definition and limits |
| --- | --- |
| State fractions | moving/N0, arrived/N0, terminated/N0; N0=0 -> undefined |
| Centroid (m) | Mean true position of living agents |
| Spread (m) | RMS distance of living agents from their centroid; one living agent -> 0 |
| Polarization | norm(mean(v / norm(v))) over living agents with speed > 1e-9 m/s; record moving-vector count; no such agent -> undefined |
| Mean ground speed (m/s) | Mean norm(velocity) over living agents, including stopped ones |
| Nearest neighbor distance (m) | Median of each living agent's closest living neighbor in the same group; fewer than 2 -> undefined |
| Minimum body gap (m) | min(norm(pi-pj) - ri-rj) across all living pairs, including different groups; one world-level value per frame, store separately or explicitly repeat as global |
| Gather target distance (m) | Mean living-agent distance to command.center; gather only |
| Gather inside fraction | Living agents within preferred radius / N0; overflow is allowed by the model, so this is descriptive, not a universal pass condition |
| Route error (m) | For living route agents: distance to nearest point on any segment of their assigned route, median and maximum; a single-point route uses point distance |
| Corridor excess (m) | max(0, route distance - route radius); legal detours can produce nonzero values |

Use a simple NumPy pairwise distance calculation for the small starter runs, with an explicit O(N²) memory note and a guard that disables pair metrics with a warning above a configurable pair-memory budget. Do not silently subsample safety metrics. No KD-tree dependency yet.

Aggregate frame metrics into final-state values, maximum terminated fraction, minimum sampled body gap and time to 90% arrived (gather/route only, threshold based on N0). If the threshold is never reached, store null with reason `not_reached_by_horizon`, not horizon time or infinity. Report its completion rate alongside statistics conditional on completion. Endpoint distances between sampled frames can miss swept contacts; termination history is the available collision outcome signal. Do not call a positive sampled body gap proof of collision freedom.

For each parameter combination aggregate across seeds: n planned/successful/failed, mean and sample standard deviation (undefined for n<2), and all seed values. Use the same seeds for paired comparisons. Report exploratory variability, not confidence intervals from three seeds. Seed repetitions are replicates; consecutive time frames are not independent replicates. Keep an explicit failure table. No scalar "best settings" score.

## 8. First test and experiment suite

Separate hard correctness checks from exploratory observations. Model failures should not be hidden by loosening checks.

### A. Control runs (must pass)

1. Empty world, steps 0 and 3: valid schema, zero agents, S+1 frames, undefined flock metrics without division errors.
2. One free agent, no fields/obstacles, speed 1, dt 0.1, 20 steps: use its initial exported heading; assert `p(t) = p(0) + v(0)*t` and constant speed within 1e-9 for this short run. No dependence on a guessed seeded position.
3. One Hold agent, no fields: unchanged position, zero ground speed, arrived state. Demonstrate why arrived alone is not travel success.
4. One Gather agent spawned near (-3,0), radius 0.001, target (0,0), target radius 1, 100 steps: arrived and final distance below 0.05 m. No collision. This protects the Fly center regression.
5. Repeat the same noisy small run twice using the same executable and args except trace path: parsed raw simulation outputs identical. Exclude Python metadata timestamps/timing. Do not promise cross-platform bitwise equality.

### B. Exploratory runs (show results; do not demand a desired outcome)

Use dt=0.1, speed=1, N=20, spawn disk radius=3, 200 steps, seeds [7,42,99] unless overridden; other parameters match the explicit example above.

1. Free Boids alignment sweep [0,1,3], same seeds: polarization, spread, body gaps and termination fractions. Nine trials; no assertion that higher alignment is always better.
2. Gather baseline: spawn (-4,0), radius 1, target (4,0), preferred radius 1, 200 steps. Compare target distance and arrival/termination fractions over time. Three trials.
3. Gather with a rectangle: same case plus obstacle at (0,0), local X extent 1, local Y extent 3, zero rotation; same seeds. Three trials. Use `--rectangle 0 0 1 3`. Plot the body boundary and trajectories; do not classify every termination as obstacle contact.
4. Bent route: spawn (-4,0), radius 1, waypoints (-4,0), (0,0), (3,3), full width 2, 200 steps, same seeds. Three trials. Show corridor, route error and surviving trajectories.

After these work, add a two-group directed interaction example and matched wind/noise comparisons as a separate milestone. Do not start with thousands of agents or a high-dimensional grid. For later count experiments, distinguish constant spawn area from constant density (radius proportional to sqrt(N)). For later dt comparisons hold physical horizon fixed; turn_response is per step, so changing dt also changes the effective controller and is not a pure numerical convergence test.

## 9. First visuals and report

Use Matplotlib with a noninteractive backend. Save PNG at readable resolution; SVG optional. No server needed. Trajectory plots use equal XY aspect, SI labels, group colors from rgba, distinct start/end markers, real body radii for a small number of final agents, rotated obstacles, fields and the active route corridor/target area. End paths at termination and mark the endpoint; do not imply terminated agents remain moving. A sampled trajectory is a visualization, not a new collision detector.

`dynamics.png` has time panels for state fractions, polarization, spread and speed; add task distance or route error when applicable. Plot undefined segments as gaps and label why. The alignment-sweep figure shows per-seed points plus mean and standard deviation; do not hide individual seeds behind a smooth trend.

Suite report.md embeds relative plot links and contains the experiment question, fixed/varied parameters, counts of runs and failures, seed-level table, aggregate metrics, observed patterns, and limitations. Use neutral computed statements; do not automatically claim scientific validation or improvements without evidence. Explain that these are simplified agents, not calibrated real quadcopters. One representative trajectory is chosen by the smallest seed, not by best outcome.

## 10. Modules and implementation order

Keep modules small and use plain functions/dataclasses; no plugin framework.

```text
analysis/pyproject.toml
analysis/README.md
analysis/swarm_analysis/__main__.py   # argparse and orchestration
analysis/swarm_analysis/experiments.py # manifest validation and expansion
analysis/swarm_analysis/runner.py      # process/files/provenance/status
analysis/swarm_analysis/reader.py      # streaming validation and geometry normalization
analysis/swarm_analysis/metrics.py     # numerical definitions and aggregation
analysis/swarm_analysis/plots.py       # figures and Markdown report
analysis/experiments/{smoke,first_experiments}.json
analysis/tests/                        # small fixtures and tests
```

1. Reader and metrics: hand-authored tiny fixtures; unit tests with known answers. Verify parallel/opposite velocities give polarization 1/0, stationary set gives undefined, points (-1,0),(1,0) have RMS spread 1 and nearest distance 2; mixed radii yield correct body gaps. Test rectangle rotation, nearest segment projection, zero-length/single-point routes, empty sets, terminated exclusion.
2. Runner and manifest: fake executable for failure/timeout/invalid JSON, deterministic grid order, unknown fields, paths with spaces, existing-output protection. Test group options stay ordered and scene bytes are copied.
3. Rust integration: build once; execute control runs; real JSONL validation; corrupted/truncated fixtures must fail visibly. No mocked integration success.
4. First experiment suite and plots: run all 18 exploratory trials, generate report, manually inspect at least one plot of each type; check obstacles are not transposed and circles are not ellipses.
5. Documentation and delivery: README gives fresh-checkout install/build/run/report commands, generated artifact locations and limitations. Commit code and tiny fixtures, never generated traces or virtual environments. Commit using existing user identity; no AI coauthor. Report exact tests run and artifact paths.

Acceptance: another developer can execute the documented commands from a fresh checkout, obtain controls that pass, all planned trial outcomes (including failures), CSVs, and the first PNG/Markdown report. Rust physics and UI remain unchanged by this Python implementation. Stop at this milestone; list optional next work separately.
