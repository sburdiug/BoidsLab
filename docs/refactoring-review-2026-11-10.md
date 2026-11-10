# Refactoring review — 2026-11-10

This bounded review covers neighbor queries, collision candidate construction,
CLI output and viewer inspectors. It preserves the Boids controller, parameters,
public API and `boids-2d-v14` / schema 2 output. No dependencies were added.

## Changes

- Reuse neighbor and collision-candidate buffers across agents instead of allocating
  a fresh vector for every query. Buffers remain local to a calculation pass.
- Filter distant neighbors and duplicate collision pairs before sorting. Ascending
  agent order is retained for floating-point accumulation and contact-event ordering.
- Share the visibility-policy filter between steering and movement reconciliation.
- Reuse rectangle serialization and lifecycle-state formatting in CLI JSON.
- Replace repeated inspector label calls with small local lists, preserving text,
  positions and formatting.

The spatial query tests now reuse the same output buffer across queries, including
boundary coordinates and the brute-force fallback. Collision tests compare contact
events as well as final agent states against the all-pairs reference.

## Measurements and equivalence

Windows, release builds, locked dependencies, seed 42, default model parameters.
Five runs per version and case, alternating before/after order. Values are medians
of each run's mean step time; initialization, output and rendering are excluded.
Baseline code: `97563f7`; the subsequent `4c4f80d` changes only README.

| Agents | Steps | Spawn radius (m) | Command | Before (ms/step) | After (ms/step) | Speedup |
| --- | --- | --- | --- | --- | --- | --- |
| 1000 | 100 | 30 | flock | 0.706 | 0.612 | 1.15x |
| 5000 | 30 | 50 | flock | 8.582 | 7.651 | 1.12x |
| 1000 | 30 | 30 | gather | 2.400 | 1.836 | 1.31x |
| 100 | 100 | 3 | cluster | 0.300 | 0.237 | 1.27x |

All paired benchmark state fingerprints match. Nine additional 300-step CLI cases
produce byte-identical summaries and full traces: empty scene, flock, gather,
spread, hold, polyline route, two rotated obstacles, wind plus measurement noise,
and two groups with asymmetric ignore/avoid policies.

Validation passed: 114 Rust tests (including viewer tests), 23 Python tests,
Clippy with warnings denied, formatting and diff whitespace checks.

Use `cargo build --release --example benchmark` and then, for example,
`target/release/examples/benchmark 1000 100 30 flock` at each revision to reproduce
the timing method. Local raw measurements, binaries and comparison script are in
the ignored `work/refactor-20261110/` directory. These short measurements establish
neither a population limit nor GUI FPS, and bitwise equivalence is environment-specific.

## Remaining candidates

- Dense spatial buckets and repeated chronological collision scans still need
  pairwise work. Profile a representative congested scene before changing the index.
- Spawn validation scans existing agents; large group creation needs separate timing
  from simulation stepping before choosing a new algorithm.
- Playback materializes interpolated positions each frame. An iterator could remove
  that allocation, but changing its public return type would break existing callers.
- Python metrics intentionally retain their pair-count limit. Scaling those metrics
  is a separate analyzer change; this pass does not weaken validation or skip data.

State snapshots in `Simulation::step` remain: they support synchronous updates and
leave the previous simulation intact if a numerical error occurs. No swarm-shape
research, parameter tuning or manual GUI inspection was performed.

## Коротко українською

Повторно використовуються буфери пошуку сусідів, сортуються лише потрібні індекси,
прибрано дублювання в експорті та інспекторах. Правила руху й формати даних збережені.
У дев'яти контрольних сценах повні траєкторії та підсумки збіглися побайтно.
Час кроку в локальних замірах зменшився приблизно на 11–24%; це не вимірювання FPS.
Щільні сцени, створення великих груп і масштабування Python-метрик потребують
окремого профілювання. Параметри рою не підбиралися.
