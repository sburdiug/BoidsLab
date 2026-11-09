# Core performance — 2026-11-01

This report records the v5 optimization snapshot. The subsequent playground model and JSON schema are described in [Playground scenes](playground.md).

The first scaling pass preserves the controller and its perception rules. Work is on branch `codex/scale-core`; no dependencies were added. These timings measure the core, not rendering FPS.

## Measurements

Windows x86-64, Rust 1.99.1, locked dependencies, release build, seed 42, default physical parameters. Each row is the median of three runs. Step timing excludes compilation, initialization and output. p95 is the median of each run's p95; the short runs have only 20 samples.

| Agents | Spawn radius | Scenario | Steps | Before, mean ms/step | After, mean ms/step | Before/after p95 ms | Speedup |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1000 | 3 | Fly towards (10, 0), preferred radius 3 | 20 | 84.84 | 31.25 | 87.39 / 32.48 | 2.7x |
| 1000 | 30 | Same destination, sparse initial placement | 20 | 25.85 | 3.48 | 27.37 / 3.96 | 7.4x |
| 5000 | 30 | Same destination | 20 | 513.56 | 36.98 | 601.54 / 43.91 | 13.9x |
| 1000 | 3 | Gather around (0, 0), preferred radius 1 | 200 | 23.95 | 21.03 | 29.93 / 26.79 | 1.1x |

The longer center-gathering scenario has a smaller gain: many agents park and the remaining local search still costs time. After 200 steps both versions have 886 hovering, 114 moving and zero terminated agents. This is not a claim that every 1000-agent scenario finishes gathering.

State fingerprints (positions, velocities, accelerations and lifecycle states) match the original version in every paired run: `7d6ea305a929cb3d`, `fe4300a19b4cad0d`, `bc14a687214983d6`, `0d834223d0c95d7b`, respectively. They are regression evidence in this numerical environment, not a cross-platform hash contract. Raw measurements: [CSV](benchmarks-2026-11-01.csv).

The original unoptimized debug build took about 253 ms/step for the first five dense flight steps. `[profile.dev] opt-level = 2` now makes ordinary IDE/Cargo development runs optimized too. Use release builds for comparisons.

## Changes

- A uniform spatial grid finds visible neighbors once per moving agent per step. Indices are sorted, preserving the previous accumulation and tie-breaking order. Exact distance checks remain; no neighbor-count cutoff was introduced.
- Collision candidates use a separate swept broad phase with reach `2 * body_radius + 2 * speed * dt`. Guard cells cover rounded inclusive boundaries. Exact chronological contact resolution remains, including contacts between agents outside each other's perception. Extreme coordinate/scale cases fall back to scanning.
- Goal search rejects candidates by a cheap lower cost before checking occupancy and parked-body paths. An unobstructed free Gather center is already the minimum, so its surrounding samples need not be searched. Ring offsets are computed once per search.
- A velocity candidate whose base cost cannot beat the current winner skips its rollout: avoidance penalties are nonnegative. The actuator, sample set and prediction horizon are unchanged.
- Playback processes at most eight steps per frame and yields at a step boundary after eight milliseconds of work. A single step can take longer than that budget. Pending model time is retained; the UI displays **Lag** when it falls behind. Under sustained overload, model playback slows relative to wall time.

## Module map

This table describes the historical v5 layout. For the current split of the core, CLI and viewer, see the [code map](architecture.md).

| Source | Responsibility |
| --- | --- |
| `src/simulation.rs` | Public types, validation, initialization, commands, atomic fixed step and baseline Boids steering |
| `src/simulation/copter.rs` | Local goal search, velocity selection and jerk-limited actuator |
| `src/simulation/collision.rs` | Swept contact broad phase and chronological event resolution |
| `src/simulation/spatial.rs` | Spatial buckets and deterministic neighborhood queries |
| `src/simulation/math.rs` | Shared vector operations |
| `src/playback.rs` | Render interpolation and bounded wall-time catch-up |
| `examples/benchmark.rs` | Repeatable core timing and state fingerprints |

At this optimization stage, public simulation imports and CLI JSON version `boids-2d-v5` remained unchanged. Scenes were added in v6; wide-corridor tracking uses `boids-2d-v7`.

## Reproduce

Build once, then time the executable so compilation is excluded:

```powershell
cargo build --release --example benchmark --locked --offline
.\target\release\examples\benchmark.exe 1000 20 3 gather
.\target\release\examples\benchmark.exe 1000 20 30 gather
.\target\release\examples\benchmark.exe 5000 20 30 gather
.\target\release\examples\benchmark.exe 1000 200 3 cluster
```

Repeat three times per input. `flock` and `hold` are also supported. The benchmark fails on invalid input or a core error and reports initialization separately. The original source and executable used locally are retained under ignored `work/optimization`; those scratch files are not required to run the current benchmark.

```powershell
cargo test --all-features --locked --offline
cargo clippy --all-targets --all-features --locked --offline -- -D warnings
cargo fmt --check
cargo run --release --features viewer --bin viewer
```

## Remaining limits

At the time of these v5 measurements the viewer limit was 1000; it has since been removed. The 5000-agent figures are headless core measurements. Increasing spawn radius changes density and behavior, so sparse and dense rows are deliberately separate. These measurements do not establish UI FPS, peak memory, a maximum supported count or worst-case latency.

Initialization still checks placement pairwise. If all agents see each other, the grid cannot remove their actual interactions: local goal search can still cost `O(N * k²)` for `k` visible neighbors, up to cubic work in a fully dense scene. Collisions, controller jams and poor parameters remain possible. Rendering and physics still share a thread; sustained overload may need a separate simulation worker, after profiling the actual workload. No GPU, threads, neighbor truncation or approximate avoidance was added.

[Ukrainian version](performance.uk.md).
