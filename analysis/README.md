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
- The analysis supports schema 2 / `boids-2d-v13`, fixed groups and static commands. Use runner `--trace-every N` to save every Nth step; initial/final frames and all intervening events remain mandatory. UIDs are scoped to one run.
- Validation and CSV calculation stream every recorded frame. Plot storage is capped at 1,000 frames and approximately 100,000 agent observations (at least initial/final frames). Plot lines connect saved points; they do not reconstruct unsampled motion. The final JSON summary and its full event history still live in memory. Pair distances remain quadratic, with calculations skipped when the living population squared exceeds 2,000,000; CSV reasons and report warnings expose every omission. This is not yet validated for very large populations.
- Cross-platform bitwise reproducibility is not promised. A supplied binary's source revision cannot be verified from its hash; its origin is marked unknown.
- Reports retain planned, valid, failed/incomplete and completed counts, even for all-failed combinations. Completion rate uses valid applicable observations only: Free Boids/Hold are null, valid unfinished arrival tasks are zero. Reports show termination, RMS spread, sampled world body gap, arrival time and polarization; they do not select an optimal setting. `process_seconds` includes initialization and output, `analysis_seconds` covers validation/metrics, and `plotting_seconds` covers per-run figures. Suite report rendering is separate. None is GUI FPS or core-only timing.


## Output interpretation (analysis version 2)

New experiment profiles use a **0.10 m radius / 0.20 m diameter**. Historical files are preserved; rerun into a new directory to compare body sizes. Reanalysis never changes simulator output or its physical settings.

CSV includes run hash, case, swept parameters, analysis version and SI units. Distances (`spread`, `nearest_neighbor`, `min_body_gap`, target/route errors) are metres; speed is m/s; time is seconds; state fractions and polarization are dimensionless. `nearest_neighbor` is the median of each living agent's nearest living neighbor **within its group**. `min_body_gap` is the minimum living-body gap **across the world**, repeated on group rows. It excludes terminated markers and is not a swept-contact test. `undefined_reasons` explains blank cells. Exact observed contact-event counts are reported separately and are world-level, not per-group counts.

`time_to_90_arrived` is the first completed simulation tick with at least 90% of the original group arrived. It is reconstructed from all validated state-change events, including transitions between snapshots. `time_to_90_interval` brackets the tick with the previous tick (or [0,0] initially); it does not invent a substep arrival instant. It measures first attainment, not sustained completion. Time averages use completed trials only. Sampled minima and geometric plots still depend on trace frequency.

`invalid_output` means validation failed; `analysis_failed` means metrics/plotting failed after input validation; process timeout/failure, setup failure, interruption and unstarted trials remain distinct. Reanalysis first invalidates previous summaries/CSV/figures. Suite reports link results only when metadata and validation agree. Metadata is checked for the requested horizon and process outcome; manually supplied traces without metadata produce an explicit verification warning. Sparse standalone traces must include metadata declaring their interval.

A sample streaming check on Windows (20 stationary agents, 1,200 versus 3,600 steps) read 8.24 versus 24.71 MB of trace data; Python-traced peaks were 18.14 versus 27.05 MB with 601 versus 901 plot frames. Instrumented analysis took 2.21 versus 6.57 s. Two 200-agent, 100-step stationary cases with spawn radii 3 m and 30 m used approximately 25 MB and 1.5 s each. These are local diagnostic measurements with plotting mocked and allocation tracing enabled, not process RSS, engine benchmarks, active-swarm scaling limits or a reason to choose a spatial index yet.

## Українська вижимка

Типовий радіус — 0,10 м, діаметр — 0,20 м; старі результати зберігають свій розмір. Аналізатор потоково перевіряє записи, підтримує `--trace-every` та зберігає всі події. Для графіків кількість точок обмежена; мінімальні відстані між кадрами невідомі. Прибуття відновлюється за подіями з точністю до кроку моделі.

Час Rust, обробки та графіків вимірюється окремо. Free Boids не отримує хибні 0% прибуття: метрика не застосовується. Невдалі запуски, незавершені задачі й пропущені метрики видимі у звіті. Повторний невдалий аналіз не залишає старий успішний підсумок. Велика кількість агентів, довгі активні сценарії та 3D потребують окремої перевірки.
