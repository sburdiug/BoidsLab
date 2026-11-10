# Swarm simulator

A 2D, fixed-step multi-agent simulation with a local Boids model, an interactive playground, and a headless CLI. It is a research prototype, not a calibrated UAV simulator. The current model is `boids-2d-v13`, JSON schema 2.

## Run

```powershell
cargo run --release --features viewer --bin viewer
cargo run --release --bin simulation -- --agents 10 --steps 600 --seed 42
cargo run --release --bin simulation -- --help
```

The viewer opens an empty paused scene. In **Group**, enter the count, seed and spawn radius; select **Add [A]** and click on the field. Add several groups, select one with Prev/Next or by clicking an agent, then give that group a command. Groups can be added while running. **Flight** and **Swarm** edit the selected group's profile; pause and choose **Apply profile** to apply changes. The scene has one shared time step.

Controls include **Fly**, **Gather**, **Spread**, **Follow**, **Hover**, and **Free Boids**. A completed gesture returns to Inspect while the selected command remains active. In **Scene**, construct rectangles by dragging their width, clicking their height, then rotating and clicking; add multiple independently editable obstacles, wind and position-noise fields. A Route is a wide polyline corridor: click vertices and press Enter to save and assign, or save without a group and assign later. See [playground controls and data](docs/playground.md).

Use **Preset** to save the environment and saved paths to a new file and load them later into the same or another scene. Groups, their parameters and current commands are not part of the preset. Pause before loading; an obstacle conflicting with a living agent or active target rejects the entire load. Saving never overwrites an existing file: choose another filename. Relative paths use the program's working directory; absolute paths are also supported.

Per directed pair of groups, **React** applies social behavior and avoidance, **Avoid** applies avoidance only, and **Ignore** disables perception. Physical contact still terminates both agents even when they ignore each other. Identical behavior profiles share a color; IDs and the selection outline distinguish groups. Restart recreates seeded groups from their original spawn recipes, preserving current scene objects, saved paths, profiles and policies. Overlapping recipes or an obstacle can make restart fail; the previous scene is retained.

## Model and units

Distances and positions are in **metres**, time and `dt` in **seconds**, velocity and wind in **metres per second**. The grid shows metres per cell. A single unobstructed agent at 2 m/s travels 2 m in one simulated second. A speed of 1 m/s is 3.6 km/h. The numbers are illustrative and can be replaced with research parameters; units do not calibrate the model to a physical aircraft.

The default physical body radius is **0.10 m** (diameter **0.20 m**). The white body circle uses this world-space radius; the direction glyph is a screen marker. Explicit smaller or larger radii remain valid. Existing saved experiment outputs are not converted.

In Free Boids, each agent considers neighbors inside its omnidirectional view radius. Separation, alignment and cohesion produce direction vectors, which are normalized and added with independently configurable weights. A direction command adds a weighted direction. The resulting heading is blended with the previous heading using `turn_response` per simulation step and normalized to the configured airspeed. With no requested direction, it retains its heading. This is an agent-based model, not a force/mass model. The turn coefficient is per step, so changing `dt` without adjusting it changes turning behavior.

Obstacles are visible when the nearest rectangle surface enters the view radius. A visible rectangle crossing the intended path supplies a detour direction. Obstacle weight 0 disables planning; between 0 and 1 it blends the detour, and values above 1 have the same effect as 1. Collision geometry is independent of that weight. A nearby obstacle off the path does not hide a wall ahead. This detour is an extension to the baseline rules and does not guarantee collision-free motion.

Gather, Spread and Route extend the same agent model with temporary local targets. They can slow or stop agents. The target circle is a soft density guide, not a rigid boundary or mandatory circular formation. For these controlled commands, a local candidate check and a bounded three-pass reconciliation examine visible neighbors and walls before committing the step. They respect directed interaction policies, but add behavior beyond classical Free Boids; they are not a complete path planner or a safety guarantee. Obstacles, noise, density, high speed and limited perception can still cause collisions or jams. There are no acceleration, braking, jerk, rotor or yaw dynamics in this branch; the previous model is preserved in `codex/copter-experiment`.

Wind is a kinematic ground-velocity offset, not an aerodynamic force. Overlapping winds add; stronger wind can overcome a stationary agent. Noise is Gaussian position error, used by local perception. Physical contacts use true positions and swept disk/rectangle geometry through the whole step. Terminated agents remain markers and no longer obstruct others.

## CLI and analysis

```powershell
cargo run --release --bin simulation -- --steps 400 --rectangle 4 0 2 3 --obstacle-angle 30 --rectangle 8 2 1 2 --obstacle-angle -15 --group -3 0 10 1 42 boids --gather 12 0 1 --group -3 5 8 1 99 boids --interaction 0 1 avoid --interaction 1 0 ignore --trace run-001.jsonl
```

`--rectangle` and `--square` are repeatable; an angle following a shape applies to that shape. Group profile/command flags after `--group` apply until the next group declaration. Profiles carry forward, commands reset. `--empty` starts without agents. Without explicit groups, `--agents` retains the single-group CLI. See `--help` for options.

Run one environment with different groups, for example `cargo run --release --bin simulation -- --scene scene.swarm --group 0 0 10 2 42 boids --speed 2 --steps 200`. Explicit CLI rectangles and fields are added to the loaded environment.

Stdout contains a JSON summary with full event history. `--trace` exclusively creates JSONL with the initial and final frames and every `--trace-every N` steps (default 1); existing files are not overwritten. Frames carry incremental events, including contacts between saved snapshots. Use agent/group `uid` for identity; storage indices can change. Group parameters live in `groups`; `legacy` is only the former single-group view. See [export contract](docs/playground.md#export-format-v13) and the [Python analyzer](analysis/README.md). Traces are analysis data, not a replay player.

## Verification and limits

```powershell
cargo test --all-features --locked --offline
cargo clippy --all-targets --all-features --locked --offline -- -D warnings
cargo fmt --check
```

The tests cover fixed-step motion in SI units, the individual Boids rules, deterministic seeding, directed group policies, multi-obstacle editing and contacts, preset roundtrips, routes, noise/wind, gathering, wall perception and JSON output. Repeatable seed does not imply cross-platform bitwise equality. The viewer has no artificial agent-count cap, but dense scenes and pairwise spawn checks can be expensive; previous benchmarks in [performance.md](docs/performance.md) are historical v5 measurements and do not establish current GUI performance. There is no 3D mode, replay viewer, batch experiment runner or Python analyzer yet.

[Ukrainian version](README.uk.md) · [Architecture](docs/architecture.md) · [Playground details](docs/playground.md)
