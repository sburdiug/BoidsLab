# Playground controls and model

`boids-2d-v12` uses one unbounded 2D scene with independent groups, multiple static rectangular obstacles, wind/noise fields and saved polyline corridors. It uses metres, seconds and metres per second. Units are an interpretation of this model, not a calibration to a particular vehicle. The former copter controller is preserved in the `codex/copter-experiment` branch.

## Groups and commands

The viewer starts empty and paused. On **Group**, set count, seed and spawn radius, choose **Add [A]**, and click a center. New groups start as Free Boids with seeded positions and directions. Multiple groups can be created during playback. Selection by Prev/Next or an agent click chooses the group that receives subsequent commands. A highlighted outline marks its agents. **Flight** and **Swarm** edit its profile on pause via **Apply profile**. `dt` belongs to the scene, so choose it before the first group or create a new scene.

Each pair of groups has an independent directed policy. React enables social and avoidance responses; Avoid retains avoidance without social alignment/cohesion; Ignore disables perception and planned response. Own group always Reacts. Actual contact terminates both regardless of policies. Identical configuration profiles have identical colors; use group IDs to distinguish them. A profile hash uses FNV-1a64 with the `boids-profile-v2` tag and the ordered little-endian f64 behavior fields in `Config::profile_hash`, excluding shared `dt`, seed, spawn, command and interaction policy; signed zero is normalized. Collisions in hash or RGB are possible; alpha is 255.

Choose a group and one command:

- **Fly [V]**: drag from start to end. The end is the target area; the start indicates direction, not a compulsory waypoint.
- **Gather [G]**: drag a soft preferred circle, or click to use Target radius. Agents seek nearby accessible space; the group need not form a circle and can extend outside it.
- **Follow [L]**: hold the mouse over the canvas to update the soft target; release keeps the last target.
- **Spread [B]**: click to use Neighbor gap or drag to measure a local spacing. Valid range is `2.6 * body_radius <= gap <= neighbor_radius`. Only React neighbors add spacing pressure. There is no target perimeter.
- **Hover** stops the selected group; **Free Boids** resumes uncommanded flocking.
- **Route [P]**: click vertices, set full Route width, press Enter or **Save / assign** in Swarm. Backspace removes a vertex; Esc cancels. Save without a selected group and use **Assign path** later, or save while selected to assign immediately. A saved path survives replacing the group's current command.

Once a gesture is complete the tool returns to Inspect; the command stays active. Paused groups can be commanded before advancing with Run or Step. Arrived agents remain living and collidable; a new command reactivates them. Terminated agents remain markers and no longer obstruct others.

## Objects and field controls

For **Rectangle [O]**, drag the horizontal width and release, click a height, then rotate and click to commit. Repeat to add more. Inspect an existing rectangle to resize it using exact fields or drag its round rotation handle. Obstacle editing is atomic; the final pose cannot touch a living agent or active target. Intermediate poses during rotation are not simulated. **Clear objects** removes every rectangle and all wind/noise fields.

Wind and noise fields can be added independently. Click for typed dimensions or drag from center, then rotate and click. Width and length are metres, speed m/s and noise sigma metres; angles are degrees in UI/CLI and radians in Rust APIs. Inspect to edit or rotate an existing field. Alt-click cycles objects without selecting agents. Multiple wind vectors add. Measurement noise affects observed positions but physical contacts and field membership use true coordinates.

Restart keeps the committed scene, paths, current profiles and policies, and regenerates each group at its original spawn center and seed. Groups inserted at different times into overlapping areas may not fit simultaneously at restart; restart then fails atomically and keeps the current scene. Explicit-agent groups have no seed recipe and cannot use seeded restart. There is no scene-file loader or replay viewer yet.

## Core model

For each moving Free Boids agent, the engine computes a vector away from neighbors inside the separation radius, a mean neighbor direction, and a vector toward visible React neighbors. Each nonzero vector is normalized, multiplied by its rule weight and added. A direction command contributes a separately weighted vector. `turn_response` mixes the normalized request with the current direction per step; the result is normalized to the configured airspeed. The heading coefficient is per step, so changing `dt` changes the turn rate. Separation uses both React and Avoid neighbors; alignment/cohesion use only React. No physical forces, mass, acceleration or jerk are modeled.

The closest rectangle surface is checked within the agent's own view radius. For visible walls intersecting the intended guide, the nearest blocking rectangle supplies a detour. Obstacle weight interpolates between the original and detour directions for values 0–1 and saturates at 1; physical collisions remain active at weight 0. For controlled commands, a local one-step candidate check uses visible neighboring agents and walls, and may reduce speed or choose a different direction. Limited view and instantaneous steering mean this remains a heuristic, not guaranteed physical safety.

Gather and route-end parking select local free-space candidates. For a region with radius `R` and `N` living group members, the preferred spacing is `max(separation_radius, 2.6 * body_radius, min(0.8 * R * sqrt(pi/N), 0.9 * view_radius))`, enlarged where another group's body is larger. The closest usable part of the region is a starting point; occupied candidates and paths through parked agents carry penalties. A temporary Gather candidate is retained while unoccupied, cleared on a new command, and reevaluated if blocked. This is not globally optimal placement.

Route progress is per agent. A route is the union of segment bands and round joins/end caps; the stored radius is half the user-facing width. Centering nudges targets toward the axis. Cohesion can slow leaders based on visible React followers. Agents can leave the corridor to avoid objects or through wind; no hard boundary or formation slots exist.

Controlled Gather/Spread/Route requests use the same three rules plus local target guidance. Their requested velocities undergo up to three deterministic passes that compare locally visible neighbors' proposed velocities for imminent contacts, choosing alternate headings/speeds where possible. Ignore groups remain invisible to this planning. This layer is an extension of classical Free Boids; it does not guarantee avoidance in every multi-agent configuration. Instantaneous command stop/turn and target arrival are simplified policies, not copter dynamics.

Wind adds to ground velocity sampled at the step start: `ground_velocity = air_velocity + wind`; `position_next = position + ground_velocity * dt`. Controlled motion can compensate wind up to its airspeed limit. Noise is zero-mean Gaussian position error per axis, deterministic from group seed, local agent index, step and field index. Reading data does not consume randomness. Field membership and collisions use true position; noisy perception may differ from the UI's true-position neighbor lines. A failed numerical step preserves the previous state and clock.

## CLI and output

```powershell
cargo run --release --bin simulation -- --steps 400 --rectangle 4 0 2 3 --obstacle-angle 30 --rectangle 8 2 1 2 --obstacle-angle -15 --group -3 0 10 1 42 boids --gather 12 0 1 --group -3 5 8 1 99 boids --interaction 0 1 avoid --interaction 1 0 ignore --trace run-001.jsonl
```

`--rectangle X Y WIDTH_X HEIGHT_Y` and `--square X Y SIDE` may be repeated. `--obstacle-angle DEG` applies to the preceding shape; it may also precede the first shape. `--wind-rect X Y WIDTH LENGTH ANGLE_DEG SPEED` and `--noise-rect X Y WIDTH LENGTH ANGLE_DEG SIGMA` may be repeated. Rectangle Rust APIs use radians and `Rectangle::new(center, width, length)` where length runs along angle zero; CLI obstacle X/Y dimensions correspond to length/width before rotation.

`--group X Y COUNT RADIUS SEED boids` starts a group. Profile/command/route options after it configure that group up to the next group; profile values carry forward, commands reset. All groups share `dt`. Without explicit groups, the original `--agents` invocation remains available; `--empty` starts without agents. Run `--help` for the complete option list.

Stdout is a single v12 JSON object. `--trace` creates a new file only, containing an initial snapshot and one per completed step. It never overwrites a file. `units` names SI units; `groups` contains IDs, seed recipes, profiles, commands, colors and interaction overrides; `paths` contains saved geometry; `agents` includes `group_id`, true/measured position, ground/air/wind velocities, state and waypoint index; `scene.rectangles` contains all obstacles and `scene.rectangle` mirrors the first for compatibility. `events` contains sequence, step and time for scene, group, profile, interaction, command and state changes. An event appears once across JSONL frames; stdout summary contains the full history. State events are reported at step precision. Long recordings use memory/disk and an I/O failure may leave an incomplete line.

```python
import json
with open("run-001.jsonl", encoding="utf-8") as source:
    for line in source:
        frame = json.loads(line)
        # Analyze frame["units"], frame["groups"], frame["agents"], frame["events"].
```

[Ukrainian version](playground.uk.md).
