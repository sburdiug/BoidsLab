# Code map

The same fixed-step core serves the CLI and viewer. UI state and rendering stay outside the library. Public imports remain `rust_playground::simulation::{...}`; implementation modules are private. The refactor preserved the v8 model. Subsequent behavior changes are described in [Playground scenes](playground.md).

## Simulation core

All paths below are relative to `src/`.

| File | Responsibility |
| --- | --- |
| `simulation.rs` | Public reexports, simulation state and read-only accessors |
| `simulation/config.rs`, `simulation/agent.rs`, `simulation/error.rs` | Parameters and validation, agent state, public errors |
| `simulation/spawn.rs` | Construction and seeded initial placement |
| `simulation/groups.rs` | Stable IDs, directed perception policies, group data and color |
| `simulation/profiles.rs` | Profile hash, atomic profile edits and seeded scene restart |
| `simulation/events.rs` | Step-indexed scene, command and lifecycle event history |
| `simulation/commands.rs` | Commands and transitions between movement modes |
| `simulation/step.rs` | Atomic fixed step, controlled-velocity reconciliation, contacts and commit |
| `simulation/boids.rs` | Weighted Boids steering and local avoidance |
| `simulation/dispersion.rs` | Local repulsion towards the requested neighbor gap |
| `simulation/parking.rs` | Temporary local goals for gathering and final route parking |
| `simulation/geometry.rs` | Rotated rectangles/squares, exact shape queries and detours |
| `simulation/fields.rs` | Wind and measurement-noise zone definitions |
| `simulation/environment.rs` | Scene edits, wind sampling and deterministic observations |
| `simulation/route.rs` | Corridor geometry, progress, center bias and local speed cohesion |
| `simulation/collision.rs` | Swept contacts, chronological termination and arrival detection |
| `simulation/spatial.rs`, `simulation/math.rs` | Spatial buckets and shared vector operations |
| `playback.rs` | Playback state, interpolation and bounded wall-time catch-up |

`Simulation::step` reads one previous snapshot and commits only after all calculations succeed. Controller, observation and contact logic remain separate; changing a display control must not introduce model calculations into the viewer.

## Applications

| File | Responsibility |
| --- | --- |
| `bin/simulation.rs` | Build/run the model, manage the trace file and report errors |
| `bin/simulation/options.rs` | Help text, argument parsing and CLI validation |
| `bin/simulation/output.rs` | JSON snapshot serialization |
| `bin/viewer.rs` | Window entry point and asynchronous frame loop |
| `bin/viewer/mod.rs` | Application state, initialization and frame orchestration |
| `bin/viewer/input.rs` | Keyboard/mouse input, picking, drawing and placement gestures |
| `bin/viewer/camera.rs` | Coordinates and zoom calculations |
| `bin/viewer/controls.rs` | Playback controls, live commands and restart |
| `bin/viewer/settings.rs` | Editable model settings, scene properties panel and validation |
| `bin/viewer/objects.rs` | Scene-object editing, selection and rotation helpers |
| `bin/viewer/groups.rs` | Group selection/spawn/profile UI and saved-path assignment |
| `bin/viewer/placement.rs` | Rectangle width/height/rotation stages and field placement |
| `bin/viewer/glyphs.rs` | Agent shape and movement-direction rendering |
| `bin/viewer/canvas.rs` | Grid, agents, fields, obstacles, corridors and previews |
| `bin/viewer/inspectors.rs` | Agent/object information, notices and footer |
| `bin/viewer/theme.rs` | Window configuration, skin and shared widgets |
| `bin/viewer/tests.rs` | Viewer settings, coordinate, zoom and marker checks |

Each frame updates interpolated markers, handles input, draws the canvas, processes playback/settings/command controls, advances playback, and draws information overlays. This order preserves the existing interactions. A window below the minimum size pauses playback before processing the frame.

## Historical v8 refactor verification

The split was checked with the existing 73 tests, formatting and Clippy. Eleven saved CLI cases retained identical stdout, stderr and exit codes; a scene combining a rotated obstacle, wind, noise and a bent corridor retained all 601 JSONL frames byte for byte. A 1000-agent, 20-step release benchmark retained state fingerprint `7d6ea305a929cb3d`. These comparisons apply to the same local numerical environment; they do not establish cross-platform bitwise equality or a performance guarantee.

[Ukrainian version](architecture.uk.md).
