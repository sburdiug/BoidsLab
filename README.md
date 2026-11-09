# Swarm simulation

A fixed-step 2D simulation with a classic Boids baseline and independently controlled copters. Model version: `boids-2d-v11`. Positions use an unbounded plane; the numerical values are demonstration units, not calibrated UAV specifications.

## Run

```powershell
cargo run --bin simulation -- --agents 10 --steps 600 --seed 42 --gather 0 0 0.05
cargo run --bin simulation -- --spread 0.8 --steps 600
cargo run --bin simulation -- --max-acceleration 1 --max-braking 2 --max-jerk 8 --gather 2 3 1 --steps 600
cargo run --bin simulation -- --help
cargo run --release --features viewer --bin viewer
```

`cargo run` defaults to the headless simulation. The CLI prints one JSON summary with configuration, command, lifecycle counts and final positions, velocities and accelerations. Errors go to stderr with a failing exit status. A CLI run uses a fixed horizon and one command per group; the library accepts new commands between steps. Legacy `--agents` runs still default to ten agents; `--empty` starts without groups. Regional CLI runs start at rest, matching the viewer's initial hover. Free Boids starts with seeded random headings at the configured speed.

## Viewer

The window starts paused and empty. In **Group**, set count/seed/spawn radius, choose **New: Copters / Boids**, press **Add [A]** and click a spawn center. Adding groups is allowed while running. Select a group with Prev/Next or by inspecting one of its agents. Commands affect that group only and return the drawing tool to Inspect after completion; the assigned command keeps running.

**Flight** and **Swarm** edit the selected group's profile with **Apply profile**, available only on pause. Profile edits preserve positions and deaths, clamp velocities to new limits and reset control acceleration. **Scene** contains a rectangular obstacle and wind/noise fields. Routes can be saved without a group, then assigned from **Swarm**. [Detailed controls and schema](docs/playground.md).

Between each directed pair of groups, **React** enables social rules and avoidance, **Avoid** enables avoidance alone, and **Ignore** disables perception of that group. Default is React; own-group interaction is always React. **Physical contact still kills both agents in every policy.** Group color is derived from behavior parameters, not its ID, seed or spawn location; equal profiles have equal colors, so use IDs to distinguish groups.

| Action | Control |
| --- | --- |
| Run / pause | F5 or button |
| One exact physics step and pause | F6 or Step |
| Recreate all groups from current profiles and original spawn recipes | F8 or Restart |
| End the run while retaining its final view | F7 or Stop; Restart to run again |
| Playback rate, 0.5x–4x | Buttons or F3 / F4 |
| Pan | Right/middle drag; arrow keys |
| Zoom | Wheel over the canvas; + / - |
| Camera origin and 100% scale | Reset view or F10 |
| Select an agent | Inspect [I], then click; F11 cycles agents |
| Selected agent's neighbors | F9 |
| Clear selection/editing and cancel a gesture | Esc |

- **Fly [V]**: drag an arrow to move the preferred gathering area. Agents head towards free space near its center, without visiting the arrow's start.
- **Gather [G]**: draw a preferred circle from center to edge, or click to use **Target radius**.
- **Spread [B]**: set **Neighbor gap** and click, or drag to measure the desired spacing. This changes local spacing, with no circular destination.
- **Follow [L]**: hold left mouse to update the preferred gathering area. Releasing it leaves the last command active. Target radius controls the circle size.
- **Hover** immediately requests a stop; wind stronger than available airspeed can prevent ground-relative hovering. **Free Boids** resumes the classic baseline.

The Gather circle is a **soft preference**, not a container or capacity constraint. Agents already in a free part of the area can stay there; approaching agents seek nearby free space. The region controls approximate density without forcing a round group shape or circulation to the center. Crowding can push parking outside the circle. Arrived agents remain alive and collidable. In wind/noise fields they keep controlling their parked position and can be disturbed. Group color is retained while hovering, with a blue ring; terminated agents remain red crosses. Pentagonal markers indicate movement direction, not body orientation or yaw. The inspector distinguishes true/measured position, ground/air velocity, wind, control acceleration and route progress. Its neighbor count/lines show true geometry.

Zoom changes smoothly around the center of the view. Opposite notches restore the scale unless a limit was reached; Windows wheel units are normalized. Click the field or press Esc before keyboard navigation after editing. Minimum window size is 1000 × 760. This canvas UI has keyboard controls but no screen-reader semantics.

Playback interpolates positions between physics snapshots at render frequency, including neighbor overlays and picking. It changes neither physical states nor CLI results. Visual latency is approximately one model step (`dt / playback_rate` seconds). Pause and single-step show exact positions; termination markers show the actual contact location. Playback processes at most eight steps per frame and yields between steps after eight milliseconds of work, retaining the backlog. A single step can exceed that budget. The Lag indicator shows when model time falls behind; sustained overload slows playback relative to wall time. Interpolation does not fix low rendering FPS under load.

## Region seeking and local avoidance

There are **no preassigned parking slots, reserved points, shared arrival times or formation trajectories**. At each model step, every moving copter independently reads the same previous snapshot and computes a temporary local goal.

For Gather, the base goal is the nearest point in the usable preferred region, including the current position if already inside. Priority neighbors are parked agents or those closer to the center, with stable index tie-breaking. Local candidates include the current position and 16 samples around those neighbors. Preferred spacing is `max(separation_radius, 2.6 * body_radius, min(0.8 * radius * sqrt(PI / alive_group_count), 0.9 * neighbor_radius))`, enlarged when needed for another group's body radius. Free candidates minimize overflow beyond the soft region plus a small travel cost, with a penalty for crossing parked neighbors. Targets are temporary and unreserved; local avoidance may brake or wait if accessible candidates are blocked.

Each axis requests velocity towards that temporary goal, bounded by maximum speed and available braking/jerk. With visible neighbors, up to 25 desired velocities are evaluated: twelve directions at full and half requested speed, plus stopping. The same jerk-limited actuator is rolled out over approximately 1.5 seconds, using at most 30 prediction steps. Neighbors are predicted at constant current velocity, and the safety radius is `max(1.4 * (own_body_radius + other_body_radius), 0.75 * separation_radius)`. Predicted closing contacts receive a large cost. With no active visible avoidance source, only the direct request is used. Only current neighbors within `neighbor_radius` influence planning.

These are bounded local heuristics, not an optimal planner or a collision-free guarantee. Other agents can change their motion after a prediction. Limited perception, insufficient braking, poor parameters or a dense group can still cause contacts, queues and jams. The sampler is neither a complete search of free space nor a packing optimizer.

A copter becomes `Arrived` after velocity and acceleration remain below 0.001 times their respective limits for 0.3 model seconds, with safe clearance to visible bodies. For Gather and final route parking, it must be near the preferred circle (with one body-radius tolerance) or alongside a visible parked neighbor closer to the center. Thus parking can grow outwards beyond a small circle. At arrival, air velocity is ideally trimmed to cancel the sampled wind and control acceleration is reset. In a calm scene this is the previous zero-velocity hover. New commands reactivate waiting agents without resetting velocity or acceleration of agents already flying; terminated agents are never revived.

## Route guidance and local dispersion

Routes now apply a soft center preference: `route_centering` defaults to 0.25; 0 restores free lateral tracking. `route_cohesion` defaults to 1 and slows leading copters when visible neighbors lag behind; 0 disables this behavior. Both live under **Swarm**, apply with **Apply profile** on pause, and have matching CLI flags `--route-centering` / `--route-cohesion`. Independent waypoint progress and collision avoidance remain active. There are no fixed square/circle/diamond/triangle formation slots.

Spread requests a minimum center-to-center gap with `Command::Spread { spacing }` or `--spread GAP`. It sums repulsion from visible React neighbors closer than the requested gap; isolated agents remain where they are. The allowed range is `2.6 * body_radius <= spacing <= neighbor_radius`. Resting agents may hover at 98% of the requested gap and reactivate if a visible neighbor encroaches. Symmetric forces may leave an agent near the center while the surrounding agents expand. This is local spacing, not uniform packing or global group connectivity.

Version v9 intentionally replaces the old `--spread X Y R` ring command. JSON now exports `{"type":"spread","spacing":...}` and the two route weights in `config`. Update scripts that expected a Spread center/radius.

## Motion model and parameters

The copter state contains position `p`, air-relative velocity `v` and control acceleration `a`. Ground velocity is `v + wind`; the speed/acceleration/jerk limits below apply to controlled air-relative motion. At each step:

- `|v| <= speed`;
- `|a| <= max_acceleration` when `dot(v_previous, a) >= 0`, otherwise `|a| <= max_braking`;
- `|a_next - a_previous| <= max_jerk * dt`.

The actuator ramps towards requested acceleration within the jerk bound. A conservative future-velocity envelope `v + a * (dt + |a| / (2 * max_jerk))` prevents commanding acceleration that cannot ramp down before the speed limit. A bounded backtracking search chooses a feasible request. Semi-implicit stepping uses `v_next = v + a_next * dt` and `p_next = p + (v_next + wind) * dt`. Wind is sampled at the start of the step; field boundaries are discontinuous in this kinematic model. These are discrete bounds, not rotor, attitude, gravity or aerodynamic dynamics. Finite timestep introduces numerical error; decreasing it refines the dynamics without changing the configured acceleration/braking/jerk limits.

| Parameter | Default | Role |
| --- | --- | --- |
| `dt` | 0.1 | Model seconds per step |
| `speed` | 1 | Maximum airspeed; constant baseline Boids airspeed |
| `max_acceleration` | 1 | Speed-increasing acceleration limit, units/s² |
| `max_braking` | 2 | Speed-decreasing acceleration limit, units/s² |
| `max_jerk` | 8 | Acceleration-change limit, units/s³ |
| `neighbor_radius` | 1 | Omnidirectional perception range |
| `separation_radius` | 0.3 | Preferred copter spacing and avoidance margin; baseline repulsion radius |
| `body_radius` | 0.025 | Physical disk radius; marker size is independent |
| `separation_weight` | 10 | Baseline separation weight; regional avoidance cost strength saturates at 1 (0 disables it) |
| `turn_response` | 0.1 | Baseline Boids heading response only |
| `alignment_weight` / `cohesion_weight` | 1 / 3 | Baseline Boids alignment/cohesion only |
| `command_weight` | 1 | Regional desired-speed factor, saturating at 1; baseline direction-command weight |
| `obstacle_clearance` | 0.15 | Extra surface clearance used for planning; UI: Obstacle gap |
| `obstacle_weight` | 1 | Predicted obstacle-contact cost; 0 disables planned avoidance |

`Hold`, switching to the baseline, and collision termination are explicit discontinuous mode changes outside copter actuator bounds. The near-rest Arrived latch is an ideal stabilization policy. Baseline Boids retains constant-speed heading steering for comparison: separately normalized separation, alignment and cohesion are weighted, then blended with the previous heading. It does not use the copter acceleration model.

## Contacts and reproducibility

Physical contact at center distance equal to the sum of the two groups' body radii terminates the participants in every mode. Swept checks cover the full piecewise-linear movement inside each step, even if the agents did not perceive each other. Events are resolved chronologically; simultaneous contact takes priority over parking. Terminated positions remain as markers and no longer obstruct agents. Events within `1e-10 * min(dt, 1)` are treated as simultaneous.

A failed numerical step preserves the entire previous state and clock. Invalid/nonfinite parameters return errors. Extreme finite values can exceed the numerical range. Seeded runs repeat in the same model, dependency lockfile, command history and numerical environment; cross-platform bitwise equality is not promised.

## Library example

```rust
use rust_playground::simulation::{Agent, Command, Config, Simulation};
let agent = Agent::new([0.0, 0.0], [1.0, 0.0])?;
let mut world = Simulation::new(Config::default(), vec![agent])?;
world.set_command(Command::Hold)?;
world.set_command(Command::Gather { center: [2.0, 0.0], radius: 0.2 })?;
world.step()?;
// Read world.agents(), velocity(), acceleration(), state(), steps(), time().
# Ok::<(), rust_playground::simulation::SimulationError>(())
```

## Check and limitations

```powershell
cargo test --all-features --locked --offline
cargo clippy --all-targets --all-features --locked --offline -- -D warnings
cargo fmt --check
```

Tests cover actuator bounds, diagonal retargets, timestep refinement, independence from baseline response, free local parking around a stopped neighbor, soft-region overflow, collective reference scenarios, swept contacts, atomic failure, CLI validation, deterministic seeds and smooth playback. Scene tests additionally cover exact swept square contacts, detours, wind compensation/overpowering, deterministic sensor noise, independent route progress, restart preservation and JSONL recording. There is no real-airframe calibration, curved spline editor or experiment dashboard. The obstacle is one static rotatable rectangle; [scene details and trace schema](docs/playground.md). Spatial grids accelerate neighborhood and swept-contact queries without truncating perception. Dense local planning and pairwise initialization remain expensive. Development builds use opt-level 2. See [performance measurements and module map](docs/performance.md) for reproducible 1000/5000-agent core benchmarks and remaining limits.

[Code map and module responsibilities](docs/architecture.md).

[Ukrainian documentation](README.uk.md).

The scene clock has one shared `dt`. Restart can fail if groups added at different times originally shared the same spawn area; the previous world is preserved. There is no scene-file loader or replay player yet. CLI snapshots and event history support external analysis.
