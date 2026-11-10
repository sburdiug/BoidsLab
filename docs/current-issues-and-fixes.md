# Current issues and recommended fixes

Planning snapshot: 2026-11-10. This document records findings and requested changes; it does not implement them. Companion: [research roadmap](research-roadmap-2d-3d.md). Ukrainian summary is included below.

## Current baseline

The Rust model is `boids-2d-v13`, export schema 2. The Python analyzer produces metrics and plots for small, static CLI experiments. Eighteen exploratory runs have been generated. Successful execution is distinct from collision-free behavior or successful task completion.

The previous engine review already led to stable run-scoped agent/group UIDs, contact events with substep time, explicit event scope, task observations and optional trace sampling. Do not list those as missing features. Current storage indices still change on deletion; consumers must join by UID. Restart creates a new run.

## P1 — Correct interpretation before more experiments

### 1. Increase the default physical body radius

**Confirmed user decision:** the base agent has radius **0.10 m**, hence diameter **0.20 m**. This is a nominal body envelope for the simplified agent, not a validated model of a particular vehicle. Larger agents, including a 1 m diameter, remain valid experiment settings. Do not impose a hard minimum radius on the engine: the request concerns the default and study scenarios.

Current defaults are radius 0.025 m, diameter 0.05 m. Sources: `src/simulation/config.rs`, `src/bin/viewer/settings.rs`. The change has not yet been applied.

Implementation must reach core defaults, UI defaults, CLI help, examples, experiment manifests and documentation. Existing explicitly configured scenes should retain their stored settings. Historical results remain historical; do not silently relabel them as 0.10 m runs.

Review separation distance, spawn area, target area and obstacle gaps together. Larger bodies at unchanged population/area create a denser experiment. Separation/view parameters should remain independently editable. A narrow gap can become physically impassable; that must be reported as a scenario property rather than hidden by shrinking bodies.

Acceptance: an untouched UI and CLI agree on 0.10 m radius; the displayed body circle has 0.20 m diameter in world units; nonoverlapping default spawning still works; body contact tests verify the sum of radii; explicit old configurations still load without alteration.

### 2. Separate engine timing from analysis and plotting

In `analysis/swarm_analysis/runner.py`, the timer starts before the process and ends after `analyze`, which includes plotting. The exported `elapsed_seconds` therefore measures more than Rust process time.

Record separate Rust process, validation/analysis and plotting times. Label the Rust process measurement as including initialization and serialization. Use the existing Rust benchmark for core-only timing; do not infer GUI FPS from either measurement. Acceptance: deliberately slowing plotting does not increase reported Rust process time.

### 3. Preserve metric applicability and failed trials

The report gives Free Boids a zero `time_to_90_completion_rate` even though they have no arrival task. `summary.json` contains a reason, but `report` discards it. Use null/not applicable for non-arrival tasks and distinguish this from a valid task that missed its horizon.

The completion denominator currently includes successful analyzed runs only. Report planned, valid, failed and completed counts separately; label any conditional completion rate. When every trial fails, the aggregate can omit the combination because no group values exist. Preserve failed combinations in the summary too.

Acceptance: free flocking, three unfinished valid arrival tasks, and three failed processes produce three distinguishable report outcomes. Missing/undefined observations are never replaced with favorable zeros.

### 4. Make metric omissions visible

`frame_metrics` skips pair metrics above its fixed pair limit; `analyze` nevertheless writes an empty warnings list. An empty value can mean too few agents, no survivors or computation intentionally skipped. Export reasons and propagate them to the report. Label `min_body_gap` as a world-level measurement because it is repeated on per-group rows.

Acceptance: a skipped safety metric is visibly marked, with the limiting condition. No silent sampling is used to claim collision freedom.

## P2 — Reliability and experimental coverage

### 5. Complete the validation and failure paths

Sources: `reader.py`, `runner.py`, `__main__.py`, `analysis/tests/`.

- Validate group parameters and event times/UID references as well as position vectors. Python JSON rejection of NaN tokens alone does not reject every overflowing numeric value in every field.
- Standalone `analyze` must check the requested horizon and process outcome from metadata. Currently expected_steps is normally supplied only by `run`.
- Distinguish invalid simulator output from a failure to compute metrics or render a plot. An exception in plotting is currently classified as invalid_output.
- Preserve status and diagnostics on interruption and setup errors, including unreadable presets and duplicate expanded trials. Prevent stale derived outputs from being reported as current success after a failed reanalysis.
- Validate positive, finite timeout values. Test timeout, process failure, invalid JSON, missing final frames and stale summaries explicitly.

Existing tests cover useful small cases, including motion, Hold, Gather, seed repeatability and a truncated trace. They do not establish all failure-path or multi-group behavior. Add targeted tests rather than rewriting the package.

### 6. Support trace sampling and larger records deliberately

`read_run` stores all frames, agents and events in memory and requires consecutive steps. It cannot yet consume the Rust `--trace-every` output. Pair calculations use quadratic memory. The current prototype is suitable for the included small cases, not yet large studies.

Move validation and metric accumulation to a streaming pass; retain a bounded set of trajectory points for plots. Support declared sampling intervals, mandatory initial/final frames and all incremental events. Report sampled arrival times as interval-censored when the exact threshold time cannot be recovered. Do not fabricate intermediate positions or claim unsampled minima are exact.

Measure memory and throughput before choosing a spatial nearest-neighbor implementation. Keep dense and sparse scenes separate. A Python performance change must not change Rust neighbor rules.

### 7. Improve report usefulness and maintainability

Current reports aggregate mostly final polarization. For stopped groups this is undefined; task, safety and cohesion summaries need equal prominence. Do not choose a configuration by one attractive endpoint plot.

Correct the damaged separator text in report headings. Separate process execution, aggregation and report assembly within the existing small modules; expand densely packed statements where they obscure error handling. Add run/configuration identity to CSV output, analysis-version provenance, and explicit units/undefined reasons. Public documentation is English with Ukrainian summaries for key documents.

## Evidence, priorities and limits

The saved first suite reports zero terminated agents in its three Gather, obstacle and route trials each. Free Boids trials have 0–20% termination. These observations motivate safety and cohesion measurements; they do not establish a general performance ranking.

Recommended order: default size and metric semantics → reliability tests → small 2D experiment matrix → sampled/streamed analysis when needed → separate 3D milestone. Avoid introducing a new physics model merely to make plots look better.

This planning task changes documentation only. No new experiments, default changes or fixes are claimed here. Performance findings from short previous runs are preliminary; full scaling behavior and long-duration stability remain unverified.

## Українська вижимка

**Узгоджений базовий розмір: радіус 0,10 м, діаметр 0,20 м.** Зараз у коді ще 0,025 м. Треба синхронно змінити початкові налаштування ядра, UI, довідку CLI та нові досліди. Це нове типове значення, а не заборона інших розмірів. Старі сцени й результати не переписуємо.

Першочергові виправлення:

1. Розділити час Rust, аналізу та малювання графіків.
2. Для Free Boids показувати «не застосовується» замість 0% прибуття; відрізняти недосягнення цілі від збою запуску.
3. Не приховувати пропущені метрики й невдалі серії. Пояснювати, чому значення відсутнє.
4. Перевіряти горизонт, статус процесу, параметри та події; додати тести таймаутів, кількох груп, загибелі всього рою та пошкоджених даних.
5. Перед великими дослідами перейти до потокового читання й підтримати розріджений запис Rust без втрати подій.
6. Показувати в підсумку безпеку, виконання задачі та цілісність рою, а не лише кінцеву поляризацію.

Постійні UID, події контактів і розділення повної/приростової історії вже додані в Rust. Їх не потрібно реалізовувати повторно. Python поки залишається прототипом для малих статичних сценаріїв. У цьому документі наведено план виправлень; код не змінено.
