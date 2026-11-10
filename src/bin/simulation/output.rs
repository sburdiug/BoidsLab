use rust_playground::simulation::{
    AgentState, Command, Config, EventKind, NoiseZone, Rectangle, Route, Simulation, SpawnConfig,
    WindZone,
};
use std::{error::Error, io::Write};

fn config(out: &mut impl Write, c: Config) -> Result<(), Box<dyn Error>> {
    write!(out, "{{")?;
    let values = [
        ("dt", c.dt),
        ("speed", c.speed),
        ("turn_response", c.turn_response),
        ("neighbor_radius", c.neighbor_radius),
        ("separation_radius", c.separation_radius),
        ("body_radius", c.body_radius),
        ("separation_weight", c.separation_weight),
        ("alignment_weight", c.alignment_weight),
        ("cohesion_weight", c.cohesion_weight),
        ("command_weight", c.command_weight),
        ("obstacle_clearance", c.obstacle_clearance),
        ("obstacle_weight", c.obstacle_weight),
        ("route_centering", c.route_centering),
        ("route_cohesion", c.route_cohesion),
    ];
    for (i, (key, value)) in values.into_iter().enumerate() {
        write!(out, "{}\"{key}\":{value}", if i == 0 { "" } else { "," })?;
    }
    write!(out, "}}")?;
    Ok(())
}
fn route_fields(out: &mut impl Write, r: &Route) -> Result<(), Box<dyn Error>> {
    write!(
        out,
        "\"radius\":{},\"width\":{},\"points\":[",
        r.radius(),
        r.width()
    )?;
    for (i, [x, y]) in r.points().iter().enumerate() {
        write!(out, "{}[{x},{y}]", if i == 0 { "" } else { "," })?;
    }
    write!(out, "]")?;
    Ok(())
}
fn command(out: &mut impl Write, c: Command, r: Option<&Route>) -> Result<(), Box<dyn Error>> {
    match c {
        Command::Flock => write!(out, "{{\"type\":\"flock\"}}")?,
        Command::Hold => write!(out, "{{\"type\":\"hold\"}}")?,
        Command::Move([x, y]) => write!(out, "{{\"type\":\"move\",\"direction\":[{x},{y}]}}")?,
        Command::Gather {
            center: [x, y],
            radius,
        } => write!(
            out,
            "{{\"type\":\"gather\",\"center\":[{x},{y}],\"radius\":{radius}}}"
        )?,
        Command::Spread { spacing } => {
            write!(out, "{{\"type\":\"spread\",\"spacing\":{spacing}}}")?
        }
        Command::FollowRoute => {
            write!(out, "{{\"type\":\"route\",")?;
            route_fields(out, r.ok_or("route command without route")?)?;
            write!(out, "}}")?;
        }
    }
    Ok(())
}
fn spawn(out: &mut impl Write, s: SpawnConfig) -> Result<(), Box<dyn Error>> {
    write!(
        out,
        "{{\"count\":{},\"center\":[{},{}],\"radius\":{},\"seed\":{}}}",
        s.count, s.center[0], s.center[1], s.radius, s.seed
    )?;
    Ok(())
}
fn scene(
    out: &mut impl Write,
    obstacles: &[Rectangle],
    wind: &[WindZone],
    noise: &[NoiseZone],
) -> Result<(), Box<dyn Error>> {
    write!(out, "{{\"rectangle\":")?;
    if let Some(r) = obstacles.first() {
        let [x, y] = r.center();
        write!(
            out,
            "{{\"center\":[{x},{y}],\"width\":{},\"height\":{},\"angle_degrees\":{}}}",
            r.length(),
            r.width(),
            r.angle().to_degrees()
        )?;
    } else {
        write!(out, "null")?;
    }
    write!(out, ",\"rectangles\":[")?;
    for (i, r) in obstacles.iter().enumerate() {
        let [x, y] = r.center();
        write!(
            out,
            "{}{{\"center\":[{x},{y}],\"width\":{},\"height\":{},\"angle_degrees\":{}}}",
            if i == 0 { "" } else { "," },
            r.length(),
            r.width(),
            r.angle().to_degrees()
        )?;
    }
    write!(out, "],\"wind\":[")?;
    for (i, z) in wind.iter().enumerate() {
        let [x, y] = z.area().center();
        let [vx, vy] = z.velocity();
        write!(
            out,
            "{}{{\"center\":[{x},{y}],\"width\":{},\"length\":{},\"angle_degrees\":{},\"speed\":{},\"velocity\":[{vx},{vy}]}}",
            if i == 0 { "" } else { "," },
            z.area().width(),
            z.area().length(),
            z.area().angle().to_degrees(),
            z.speed()
        )?;
    }
    write!(out, "],\"noise\":[")?;
    for (i, z) in noise.iter().enumerate() {
        let [x, y] = z.area().center();
        write!(
            out,
            "{}{{\"center\":[{x},{y}],\"width\":{},\"length\":{},\"angle_degrees\":{},\"position_stddev\":{}}}",
            if i == 0 { "" } else { "," },
            z.area().width(),
            z.area().length(),
            z.area().angle().to_degrees(),
            z.position_stddev()
        )?;
    }
    write!(out, "]}}")?;
    Ok(())
}
fn state(s: AgentState) -> &'static str {
    match s {
        AgentState::Moving => "moving",
        AgentState::Arrived => "arrived",
        AgentState::Terminated => "terminated",
    }
}

pub(super) fn write_snapshot(
    out: &mut impl Write,
    world: &Simulation,
    seed: u64,
    spawn_radius: f64,
    reason: &str,
    event_start: usize,
    full_history: bool,
) -> Result<(), Box<dyn Error>> {
    let count = world.agents().len();
    let measured: Vec<_> = (0..count)
        .map(|i| world.measured_position(i))
        .collect::<Result<_, _>>()?;
    let states = |s| world.agents().iter().filter(|a| a.state() == s).count();
    write!(
        out,
        "{{\"model\":\"boids-2d-v14\",\"schema_version\":2,\"event_scope\":\"{}\",\"units\":{{\"length\":\"m\",\"time\":\"s\",\"speed\":\"m/s\"}},\"completion_reason\":\"{reason}\",\"steps\":{},\"time\":{},\"initial_agents\":{count},\"moving\":{},\"arrived\":{},\"terminated\":{},\"legacy\":{{\"seed\":{seed},\"spawn_radius\":{spawn_radius},\"config\":",
        if full_history {
            "full_history"
        } else {
            "incremental"
        },
        world.steps(),
        world.time(),
        states(AgentState::Moving),
        states(AgentState::Arrived),
        states(AgentState::Terminated)
    )?;
    config(out, world.config())?;
    write!(out, ",\"command\":")?;
    command(out, world.command(), world.route())?;
    write!(out, "}},\"groups\":[")?;
    for (i, g) in world.groups().iter().enumerate() {
        write!(
            out,
            "{}{{\"id\":{},\"seeded\":{},\"profile_hash\":\"{:016x}\",\"rgba\":{:?},\"default_interaction\":\"React\",\"spawn\":",
            if i == 0 { "" } else { "," },
            g.id(),
            g.is_seeded(),
            g.profile_hash(),
            g.color()
        )?;
        spawn(out, g.spawn())?;
        write!(out, ",\"config\":")?;
        config(out, g.config())?;
        write!(out, ",\"command\":")?;
        command(out, g.command(), g.route())?;
        write!(out, ",\"interactions\":[")?;
        for (j, (target, mode)) in g.interactions().iter().enumerate() {
            write!(
                out,
                "{}{{\"target\":{target},\"mode\":\"{mode:?}\"}}",
                if j == 0 { "" } else { "," }
            )?;
        }
        write!(out, "],\"uid\":{}}}", g.uid())?;
    }
    write!(out, "],\"paths\":[")?;
    for (i, r) in world.paths().iter().enumerate() {
        if i > 0 {
            write!(out, ",")?;
        }
        write!(out, "{{\"id\":{i},")?;
        route_fields(out, r)?;
        write!(out, "}}")?;
    }
    write!(out, "],\"events\":[")?;
    for (i, (sequence, e)) in world
        .events()
        .iter()
        .enumerate()
        .skip(event_start)
        .enumerate()
    {
        write!(
            out,
            "{}{{\"sequence\":{sequence},\"step\":{},\"time\":{},",
            if i == 0 { "" } else { "," },
            e.step,
            e.time
        )?;
        write!(
            out,
            "\"group_uids\":{:?},\"agent_uids\":{:?},",
            e.group_uids, e.agent_uids
        )?;
        match &e.kind {
            EventKind::AgentContact { first, second } => write!(
                out,
                "\"type\":\"agent_contact\",\"first\":{first},\"second\":{second}"
            )?,
            EventKind::ObstacleContact {
                agent,
                obstacle,
                shape,
            } => {
                write!(
                    out,
                    "\"type\":\"obstacle_contact\",\"agent\":{agent},\"obstacle_index\":{obstacle},\"geometry\":"
                )?;
                scene(out, &[*shape], &[], &[])?;
            }
            EventKind::GroupRemoved { group, agents } => write!(
                out,
                "\"type\":\"group_removed\",\"group\":{group},\"agents\":{agents}"
            )?,
            EventKind::GroupAdded {
                seeded,
                group,
                config: c,
                spawn: s,
                command: cmd,
            } => {
                write!(
                    out,
                    "\"type\":\"group_added\",\"seeded\":{seeded},\"group\":{group},\"config\":"
                )?;
                config(out, *c)?;
                write!(out, ",\"spawn\":")?;
                spawn(out, *s)?;
                write!(out, ",\"command\":")?;
                command(out, *cmd, None)?;
            }
            EventKind::ProfileChanged { group, config: c } => {
                write!(
                    out,
                    "\"type\":\"profile_changed\",\"group\":{group},\"config\":"
                )?;
                config(out, *c)?;
            }
            EventKind::CommandChanged {
                group,
                command: c,
                route: r,
            } => {
                write!(
                    out,
                    "\"type\":\"command_changed\",\"group\":{group},\"command\":"
                )?;
                command(out, *c, r.as_ref())?;
            }
            EventKind::InteractionChanged {
                source,
                target,
                mode,
            } => write!(
                out,
                "\"type\":\"interaction_changed\",\"source\":{source},\"target\":{target},\"mode\":\"{mode:?}\""
            )?,
            EventKind::StateChanged { agent, from, to } => write!(
                out,
                "\"type\":\"state_changed\",\"agent\":{agent},\"from\":\"{}\",\"to\":\"{}\"",
                state(*from),
                state(*to)
            )?,
            EventKind::PathAdded { path, route: r } => {
                write!(out, "\"type\":\"path_added\",\"path\":{path},")?;
                route_fields(out, r)?;
            }
            EventKind::PathsReplaced { paths } => {
                write!(out, "\"type\":\"paths_replaced\",\"paths\":[")?;
                for (i, path) in paths.iter().enumerate() {
                    if i != 0 {
                        write!(out, ",")?;
                    }
                    write!(out, "{{")?;
                    route_fields(out, path)?;
                    write!(out, "}}")?;
                }
                write!(out, "]")?;
            }
            EventKind::SceneChanged {
                obstacles,
                wind,
                noise,
            } => {
                write!(out, "\"type\":\"scene_changed\",\"scene\":")?;
                scene(out, obstacles, wind, noise)?;
            }
        }
        write!(out, "}}")?;
    }
    write!(out, "],\"scene\":")?;
    scene(
        out,
        world.obstacles(),
        world.wind_zones(),
        world.noise_zones(),
    )?;
    write!(out, ",\"agents\":[")?;
    for (id, agent) in world.agents().iter().enumerate() {
        let state = match agent.state() {
            AgentState::Moving => "moving",
            AgentState::Arrived => "arrived",
            AgentState::Terminated => "terminated",
        };
        let [x, y] = agent.position();
        let [vx, vy] = agent.velocity();
        let [avx, avy] = agent.air_velocity();
        let [wx, wy] = agent.wind_velocity();
        let [mx, my] = measured[id];
        let comma = if id == 0 { "" } else { "," };
        write!(
            out,
            "{comma}{{\"group_id\":{},\"id\":{id},\"position\":[{x},{y}],\"measured_position\":[{mx},{my}],\"velocity\":[{vx},{vy}],\"air_velocity\":[{avx},{avy}],\"wind_velocity\":[{wx},{wy}],\"waypoint_index\":{},\"state\":\"{state}\",\"uid\":{},\"group_uid\":{},\"task_observation\":",
            agent.group_id(),
            agent.waypoint_index(),
            agent.uid(),
            world.group(agent.group_id()).unwrap().uid()
        )?;
        let group = world.group(agent.group_id()).unwrap();
        let target = match group.command() {
            Command::Gather { center, radius } => Some((center, radius)),
            Command::FollowRoute => group
                .route()
                .map(|r| (*r.points().last().unwrap(), r.radius())),
            _ => None,
        };
        if let Some((center, radius)) = target {
            let distance = (x - center[0]).hypot(y - center[1]);
            write!(
                out,
                "{{\"target_distance\":{distance},\"inside_preferred_area\":{}}}",
                distance <= radius
            )?;
        } else {
            write!(out, "null")?;
        }
        write!(out, "}}")?;
    }
    writeln!(out, "]}}")?;
    Ok(())
}
