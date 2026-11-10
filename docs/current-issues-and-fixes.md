# Fix status and remaining limits

Updated 2026-11-10 after implementing the recommendations from this document. Model: `boids-2d-v13`, export schema 2, analysis contract version 2. The [research roadmap](research-roadmap-2d-3d.md) remains a separate plan; 3D and new research scenarios were not implemented in this fix pass.

## Completed fixes

| Finding | Implemented change | Verification |
| --- | --- | --- |
| Default bodies were only 0.025 m in radius | Core, viewer, CLI help and new experiment profiles use **0.10 m radius / 0.20 m diameter**. Other radii remain valid. | UI/core/CLI agree; default spawning is nonoverlapping; swept contact occurs at the sum of radii; explicit 0.025 m CLI settings remain intact. Small-body historical test fixtures now specify their radius. |
| Rust timing included analysis and plots | Separate `process_seconds`, `analysis_seconds`, `plotting_seconds`. Process time includes initialization and serialization. | A delayed plot leaves recorded process time unchanged. Suite-level report rendering is outside per-run timings. |
| Free flocking appeared to have 0% completion; failed combinations disappeared | Null/not applicable for Free Boids/Hold; zero only for valid arrival tasks that miss the horizon. Planned, valid, failed/incomplete, applicable and completed counts remain visible, including all-failed combinations. | Distinct free/unfinished/failed report tests; interruption preserves unstarted trials. |
| Pair metrics silently disappeared | CSV reasons plus report warnings identify the 2,000,000 pair-matrix limit, insufficient population and inapplicable metrics. World body gap is explicitly world-scoped. | Pair-limit, all-dead and multiple-group UID tests. |
| Incomplete validation and ambiguous errors | Reject nonfinite/overflowing numbers, invalid parameters, references, event order/state histories and missing frames. Check process outcome and horizon from metadata. Distinguish invalid output, computation/plot failure, setup failure, timeout and interruption. Reject duplicate grid trials and invalid timeouts. | Targeted failure tests, stale-result invalidation, sparse final-frame/event consistency. |
| Entire trace retained in memory; sparse traces rejected | Stream validation and metrics into CSV. Retain bounded plot observations; consume declared sparse intervals and all events. First 90% arrival is reconstructed from state events at completed-tick resolution, including transient attainment between snapshots. | Sparse trace with nonmultiple final step; event reconstruction across state reversals; 2,100-step plot-buffer test. |
| Reports focused on polarization; provenance and organization weak | Separate execution, analysis and reporting. Add safety/task/spread summaries, clean headings, run/configuration/version/units in CSV, per-cell reasons, copied manifests and binary hashes. Preserve the alignment sweep figure. | Real control suite and 18 fresh experiment runs; report/plot inspection. |

The on-screen white body circle already uses `body_radius * camera_zoom`, so the new default has a 0.20 m world diameter. Direction glyphs remain screen markers. The viewer was not manually exercised in this pass; its relevant configuration tests passed.

## Checks and observations

- Full Rust tests, formatting and Clippy with warnings denied.
- Python unittest controls plus targeted reliability tests: motion, Hold, Gather, deterministic noise, corrupt/truncated output, finite data, metadata, sparse sampling, UID grouping, metric omissions, timing, process failure/timeout/setup/interruption and stale summaries.
- The existing 18-case experiment matrix was rerun with 0.10 m bodies and `--trace-every 5`, separately from historical output, at `work/analysis/body010-fixes/`. All 18 processes and analyses completed. This is not a claim that every navigation task succeeded.
- Those runs show zero termination in the controlled Gather/obstacle/route cases but no 90% arrival attainment by their configured horizon. Free-flocking cases show substantial termination. These are exploratory observations, not a calibrated safety assessment, an optimal setting or a regression diagnosis. Larger bodies changed the experiment; defaults were not tuned to hide collisions.
- Local streaming diagnostics are recorded in `work/analysis/streaming-check/measurements.json` and summarized in the [analyzer README](../analysis/README.md). A 3x longer 20-agent trace used approximately 1.5x the Python-traced peak, with growth explained by retained plot frames. Dense and sparse 200-agent stationary cases were checked separately. Plotting was mocked; allocation tracing adds runtime overhead. These are not engine/FPS or full-process-memory benchmarks.

## Remaining limits

1. The final Rust summary still contains the complete event history and is held as one complete document; plotting holds a bounded sample. Memory is not strictly independent of event count or population. Pair distances still use a bounded quadratic calculation. No new spatial nearest-neighbor implementation was justified by this small diagnostic run.
2. Supported input remains static CLI groups, commands and environment. A supplied binary's hash does not prove which source built it. The export model version is unchanged because the schema and equations did not change; compare complete parameters, especially radius.
3. Sampled body gaps cannot establish collision freedom between observations. Exact contact events are separate. First threshold attainment does not mean sustained convergence; arrival timestamps have simulation-step resolution, not inferred continuous precision.
4. Broad active-swarm scaling, long-duration stability, confidence intervals, additional scenarios, real vehicle calibration and all 3D work remain future research. This fix pass does not select ideal weights or promise collision-free motion.

## Українська вижимка

Виправлення реалізовано. Типовий радіус тепер **0,10 м**, діаметр **0,20 м** у ядрі, UI, CLI та нових дослідах. Старі результати й явно задані розміри не змінюються.

Аналізатор окремо вимірює час процесу, аналізу й графіків; розрізняє збій, недосягнення цілі та незастосовну метрику. Усі невдалі серії залишаються у звіті. Пропуски метрик мають пояснення. Додано потокове читання, розріджені кадри без втрати подій, обмеження пам'яті графіків та перевірки станів/посилань. Прибуття відновлюється з подій, з точністю до кроку моделі. Невдалий повторний аналіз прибирає застарілі результати.

Перевірено Rust і Python, повторно виконано 18 дослідів з новим радіусом. Успішний запуск не означає виконання задачі: контрольовані сценарії не досягли 90% прибуття за свій горизонт. Великі активні рої, тривала стійкість, нові сценарії та 3D ще не перевірені. Повна історія подій залишається в пам'яті, а попарні відстані мають ліміт обчислення.
