# Playground scenes

Model `boids-2d-v11`: one unbounded 2D scene, independently configured groups, a static rectangle, wind/noise fields and wide polyline corridors. CLI and viewer use the same fixed-step core. Units are illustrative, without real-airframe calibration.

## Build a scene

The viewer opens empty and paused. In **Group**, set the new count, seed and spawn radius, choose **New: Copters / Boids** (Boids is the default), then **Add [A]** and click a spawn center. Copters begin hovering; Free Boids begins at constant airspeed with seeded headings. Additions are allowed during playback. Existing agent/group IDs remain stable; a rejected spawn leaves the scene unchanged. Agent counts are limited by available resources and placement feasibility.

Select a group with Prev/Next or by inspecting one of its agents. **Flight** and **Swarm** show that group's editable profile. **Apply profile** works only on pause and preserves positions and terminated agents. It clamps current velocities to the new limits and resets control acceleration; this explicit edit is outside the normal jerk bound. Invalid edits preserve the current profile. New count/seed/spawn radius affect new groups only.

All groups share the scene's `dt`. Choose it before adding the first group at step zero, or use **New scene** while paused/stopped to start over. Other motion parameters belong to each group.

For each directed pair, use **To group** and choose:

| Policy | Response to the other group |
| --- | --- |
| React | Social alignment/cohesion and local spacing/avoidance |
| Avoid | Avoidance only; no social alignment/cohesion or Spread spacing request |
| Ignore | No perception or planned response |

Policies are edited on pause, can differ in opposite directions, and default to React. Own-group interaction is always React. **Physical contacts always terminate both participants, including ignored groups.** Disabling a planning weight can still disable the corresponding avoidance behavior.

Color is a deterministic profile hint, not group identity. Equal parameter profiles give equal RGB regardless of seed, count, location, current command or interaction policies. Alpha is always 255 for visibility. IDs distinguish identical profiles. The hash is FNV-1a64 over `boids-profile-v1` followed by ordered little-endian f64 behavior fields in `Config::profile_hash`; scene `dt` is excluded and signed zero is canonicalized. This is not cryptographic; hash/color collisions are possible.

## Commands and saved paths

Commands address the selected group. Completing a gesture returns the tool to **Inspect**, while the group's assigned command remains active.

- **Fly [V]**: draw an arrow towards a destination region. Its start is a direction guide, not a waypoint everyone must visit.
- **Gather [G]**: draw a soft preferred circle, or click to use Target radius. Agents occupy nearby free places without a prescribed round formation. Agents already inside free space need not migrate to the center. Local crowding can extend parking beyond the circle.
- **Follow [L]**: hold left mouse to update the destination region. Release leaves the last target active.
- **Spread [B]**: click to request Neighbor gap, or drag to measure it. Gap must be between `2.6 * body_radius` and `neighbor_radius`. Only visible React neighbors contribute the spacing pressure; Avoid neighbors still participate in predictive avoidance. Isolated agents stay in place. This is not a perimeter command or uniform packing guarantee.
- **Hover** requests an immediate stop; **Free Boids** switches to classical constant-speed steering. Both affect only the selected group.
- **Route [P]** in Scene: click vertices, set Route width, then Enter or **Save / assign** in Swarm. Backspace removes a vertex, Esc cancels the draft. Saving works without a selected group. If one is selected, saving also attempts assignment. Choose a saved Path and **Assign path** to reuse it for another group. Replacing a command clears its active route, not the saved path.

The circle controls approximate density using local free-space candidates; it does not assign fixed slots. Crowded or occluded agents can still queue or fail to settle. Arrived agents remain alive, collidable and responsive to later commands. Group color remains on their bodies; a blue ring marks waiting. Terminated agents remain red crosses and stop obstructing others.

## Objects

The rectangular obstacle is created in three stages: drag horizontal width and release, click the height, then rotate the preview and click to commit. Its two dimensions are independent. A second rectangle replaces the first; multiple solid obstacles are not yet supported.

Wind and noise retain their independent Width/Length controls: click to use typed dimensions or drag from the center to set them; release, rotate the preview and click to commit. Wind speed and noise sigma are independent of dimensions. Inspect existing objects, drag their round rotation handle, or edit exact properties and **Apply object**. Alt-click bypasses agents when picking overlapping objects. **Clear objects** removes the solid rectangle and fields.

Edits apply immediately at a frame boundary, including on pause. A final obstacle pose touching a live body or active target is rejected. Rotation/resizing is an atomic static edit; intermediate swept poses are not simulated. Fields are not solid.

**Restart** regenerates all seeded groups from their original spawn recipes using current profiles, commands, routes and policies, preserving committed geometry and saved paths. It resets time and event history. It can fail if live-added groups originally occupied the same spawn area at different times, or if changed geometry blocks placement; the previous world remains. Groups built with explicit `Simulation::new` agents are marked `seeded: false` and cannot use seeded restart. A scene-file loader and replay player are not implemented.

## Wall perception and corridors

Each group uses its own omnidirectional `neighbor_radius`. The wall is perceived when the **nearest surface**, not its center, is within that radius. Once perceived, its full static rectangle geometry is available to the controller. Group-to-group Ignore policies do not hide walls. `obstacle_weight = 0` disables planning response, but physical contacts remain active.

Copters build a local visibility-graph detour around the body plus `obstacle_clearance`, then evaluate predicted wall contacts using the jerk-limited actuator. Inside the planning gap, a tangential waypoint is retained unless the direct guide intersects the physical body. Free Boids uses a simpler heading detour; its guide extends beyond a visible wall so a target inside a thick obstacle cannot project back to its near face and suppress turning.

Physical disks use exact swept rectangle-face and rounded-corner contacts, including rotated rectangles. A contact terminates the agent. Perception and braking limits, noise, coarse steps and local minima can still cause collisions or jams: avoidance is not guaranteed.

Routes are wide soft corridors: segment bands with round joins/end caps. Route width is the full width; `Route::new(points, radius)` uses half-width. Agents enter at accessible lateral positions and advance independently. `route_centering` gently returns them towards the axis; zero permits free lateral tracking. `route_cohesion` slows leaders for visible React followers. It is suspended near a visible wall and during final parking. Avoid and Ignore groups do not socially pull the route group. There are no rigid formation slots or shared waypoint barriers. Wind and avoidance can carry agents outside the band; they try to return afterwards. Vertices overlapping the physical obstacle are rejected, but the band itself may cross it.

## Wind and measurements

Wind is a kinematic velocity field, not an aerodynamic force. Overlapping velocities add. At the start of each step, sample `w(p)`; then `v_air_next = v_air + a_next * dt`, `v_ground = v_air_next + w`, and `p_next = p + v_ground * dt`. Airspeed/acceleration/braking/jerk bounds apply to controlled air motion. Ground speed can exceed the limit. Strong wind can defeat hovering. Sharp field boundaries introduce discrete changes; reduce `dt` to examine them.

Noise adds independent zero-mean Gaussian position errors on each axis. The controller observes self and neighbors through that noise. Physical contacts, field membership and stored positions use true geometry. Velocity and wind are measured perfectly. Samples depend on the group's spawn seed, local agent index, step and field index. Reading observations or exporting does not advance randomness. Explicit-agent groups use sensing seed zero. Displayed neighbor lines use true positions filtered by directed perception; noisy planning may differ.

## CLI and analysis

```powershell
cargo run --release --bin simulation -- --empty --steps 0
cargo run --release --bin simulation -- --steps 400 --rectangle 4 0 2 3 --obstacle-angle 30 --group -3 0 10 1 42 copters --speed 1 --gather 7 0 1 --group -3 5 8 1 99 boids --speed 0.8 --interaction 0 1 avoid --interaction 1 0 ignore --trace run-001.jsonl
cargo run --release --bin simulation -- --group -3 0 10 1 42 copters --waypoint -3 0 --waypoint 7 0 --waypoint 7 3 --route-width 2 --steps 400
```

`--group X Y COUNT RADIUS SEED copters|boids` is repeatable. Profile/command/route flags following it configure that group until the next declaration. Profile values carry forward; command and route reset. All groups must use one `dt`. `--agents` cannot be combined with explicit groups. Without groups, the legacy CLI still defaults to ten Free Boids; `--empty` creates none. One group can use at most one command or route. Routes override the group's default mode.

`--rectangle X Y WIDTH_X HEIGHT_Y` creates the solid body; `--square X Y SIDE` remains an equal-dimensions alias. `--obstacle-angle` / `--square-angle` use degrees. `--wind-rect X Y WIDTH LENGTH ANGLE_DEG SPEED` and `--noise-rect X Y WIDTH LENGTH ANGLE_DEG SIGMA` are repeatable. Fields' length runs along their angle, width across it. Rust `Rectangle::new(center, width, length)` follows that field convention; CLI obstacle X/Y dimensions map to length/width at zero rotation. Rust angles are radians. Legacy square-field flags remain supported. See `--help` for motion flags.

Stdout is one JSON summary. `--trace` exclusively creates a JSONL file containing the initial snapshot and every completed step; existing files are never overwritten. The v11 authoritative fields are `groups[]` (profile, spawn, command, policies, hash, RGBA), `paths[]`, `agents[].group_id`, `scene.rectangle` (center, width, height, angle_degrees) and `events[]`. Top-level config/command are compatibility mirrors of group zero; top-level seed/spawn_radius are legacy inputs. Read each group's spawn for multi-group data. This intentionally replaces `scene.square` from older versions.

Events record group additions, profile/command/policy changes, scene edits, saved paths and lifecycle transitions with sequence, step and model time. Each event appears once across a JSONL trace; the final stdout summary includes the full history. Contact state-event timestamps are quantized to the end of the resolved step, even though physical swept contact positions are computed within it. This is not an exact substep event-time stream or a replay engine. History consumes RAM; long traces consume disk. I/O failure can leave an incomplete final line.

Agent snapshots contain ID, group_id, true/measured position, ground/air/wind velocities, control acceleration, waypoint_index and moving/arrived/terminated state. A failed numerical step preserves the previous state, clock and event history. Seeded reproducibility assumes the same model, lockfile, inputs and numerical environment; cross-platform bitwise equality is not promised.

```python
import json
with open("run-001.jsonl", encoding="utf-8") as source:
    for line in source:
        frame = json.loads(line)
        # Analyze frame["groups"], frame["agents"] and frame["events"].
```

[Ukrainian version](playground.uk.md).
