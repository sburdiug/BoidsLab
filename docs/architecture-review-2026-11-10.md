# Architecture review for the first Python analyzer

Reviewed 2026-11-10 at `b7894dbe7153f08d58f2d2b3d6721e76e22f6426`. This is a source-based review of module boundaries, simulation orchestration, group lifecycle, CLI/export, presets, playback and the analysis integration points. It is not a claim of exhaustive correctness, a security audit or fresh performance measurements.

## Decision

Keep the Rust core and call the compiled CLI from Python. The first analysis pipeline does not need bindings, a simulation service, a new ECS or a rewrite. Implement the [pipeline handoff](python-pipeline-plan.md) before extending the engine. No identified issue blocks the small static-group experiment suite when its documented restrictions are enforced.

The useful boundaries are already present: `Simulation` owns model state; steering, geometry, observation and swept contact resolution are separate modules; `Playback` owns wall-time/interpolation; the viewer owns gestures/rendering; the CLI owns process-level input/output. The Python layer should own experiment orchestration, provenance, numerical summaries and reports.

## Findings and priorities

### R1 — Mutable indices are not durable identities

Evidence: `Simulation::remove_group` in `src/simulation/groups.rs` drains agents, removes a group, and shifts all subsequent indices. `EventKind` and `output::write_snapshot` use these indices as IDs. Existing events retain their old index values. The architecture map incorrectly called them stable IDs; corrected in both languages with this review.

Impact: a future live recording containing deletion cannot safely join trajectories by numeric ID alone; a later agent can inherit an earlier agent's index. This does not affect current static CLI runs.

MVP action: reject membership changes after initialization. Later: introduce monotonic external AgentId/GroupId distinct from storage indices, preserve them across deletion and export them in events. Acceptance: delete the middle group, create another, and prove surviving trajectories and event references retain identity. Treat this as an export migration, not a cosmetic rename.

### R2 — The event stream does not support collision-cause analysis

Evidence: `src/simulation/collision.rs` has internal Contact/Obstacle events, but `step.rs` exports only lifecycle changes through `EventKind::StateChanged`. `set_group_command` in `commands.rs` changes states directly and records the command rather than individual state changes.

Impact: one cannot recover exact contact pairs, obstacle identity or substep contact time from the public output. Counts inferred only from lifecycle events can also disagree with command-induced snapshot states.

MVP action: count terminated agents from snapshots; use events for timing at step resolution. Never report "number of collisions" from termination count. Later, if required by the research question, add explicit contact events with stable IDs, contact kind and substep time, with tests for simultaneous and obstacle contacts. Preserve the existing snapshots as the authoritative state.

### R3 — Export metadata is ambiguous for multiple groups

Evidence: `write_snapshot` receives the legacy CLI seed/spawn_radius, while explicit group seeds live under `groups[*].spawn`. Top-level config/command reflect only the first group. `Options::parse` carries profile values into subsequent groups. A step-0 probe with group seeds 7 and 99 produced top-level seed 42; setting speed 2 for the first group also set the second group's speed to 2 unless overridden.

MVP action: use per-group metadata and make every supplied group's profile explicit. Later: define a separately versioned output schema, deprecate ambiguous top-level fields, and add serialized build/model provenance. The current `boids-2d-v12` label alone cannot distinguish recent behavior fixes; record executable and source hashes now.

### R4 — Trace volume and process time can mislead scaling studies

Evidence: CLI writes every agent plus repeated scene/config/group data every step. `Simulation.events` is an unbounded vector; summary includes the full event history. `ScenePreset::apply` clones the whole world, including that history. Spawn checks existing agents pairwise. `step.rs` clones state and rebuilds neighbor indexes; parking uses local candidate searches. Viewer physics and rendering share one thread, and an individual step cannot be interrupted by Playback's time budget.

Impact: large trial batches can be dominated by disk, serialization, allocation or spawn work; a responsive playback budget is not a bound on a single expensive step. Historical performance.md numbers concern an older controller and are not current throughput guarantees.

MVP action: small cases, sequential execution, streaming reads, bounded plotting samples and timeouts. Record process elapsed time as end-to-end time only. Later: profile current release workloads and separate initialization/core/export costs using `examples/benchmark.rs`; only then consider trace sampling, an event sink or a simulation worker. Sampling must retain events and final snapshots. Do not change neighbor semantics as an optimization without a model comparison.

### R5 — Geometry names and arrival states require interpretation

Evidence: `Rectangle` length is along local X; obstacle JSON calls that dimension width, while wind/noise JSON calls it length. `near_region` in collision.rs allows a stopped agent outside the target radius to arrive near a closer arrived group member. Hold also means arrived. Fly, Gather and Follow share the Gather core command.

MVP action: normalize dimensions in one reader function; test rotated asymmetric rectangles. Show both arrival fraction and geometric target distance/inside fraction. Do not impose "every agent inside the circle" as a universal correctness rule: the accepted model uses a soft area. Future model changes should separate task success from lifecycle state if strict task semantics are needed.

### R6 — Seeded restart and presets are not replay

Evidence: `restarted` rebuilds groups from their spawn recipes and current commands; ScenePreset contains environment and path library only. CLI has no schedule for command changes and no assignment flag for a saved preset path. Measurements exported at a completed step are current observations, not an explicit record of every observation consumed during the preceding integration step.

MVP action: run static-command experiments and supply route waypoints explicitly; compare output trajectories, not reconstructed controller decisions. Later: a scenario schedule, explicit initial-state import and observation/action logging should be separate features with deterministic replay tests. Do not describe current presets as complete experiment snapshots.

## Changes made during this review

- Corrected the stable-ID claim in the architecture maps.
- Added the implementation handoff and Ukrainian overview with validation and supported-scope restrictions.
- Left Rust behavior unchanged: none of the above requires a rushed physics/export migration for the first small analyzer.

## Evidence and limits

Ran `cargo test --all-features --locked --offline`: 109 tests passed. Ran the real CLI with two explicit groups at step 0 to verify profile inheritance, per-group seeds and Hold state. Inspected current implementations and existing tests; no new graphical inspection, long-duration soak, large-N benchmark, Python implementation or experiment results were produced in this design task.

Recommended order: implement the small pipeline, inspect its first plots, then decide which model/export limitation actually prevents answering the research question. Avoid treating every future capability as a prerequisite for the first useful report.
