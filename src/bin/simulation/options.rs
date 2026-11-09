use rust_playground::simulation::{
    Command, Config, Interaction, NoiseZone, Rectangle, Route, SpawnConfig, Square, WindZone,
};
use std::{error::Error, str::FromStr};

pub(super) const HELP: &str = "Usage: cargo run --bin simulation -- [OPTIONS]

Runs the shared 2D core without a window and writes one JSON summary.

  --empty                     Start with no agents
  --group X Y N R SEED MODE    Add a group; MODE: copters or boids (repeatable)
  --interaction A B MODE      Directed policy: react, avoid, ignore (repeatable)
  --rectangle X Y W H         Rectangular obstacle; world X/Y dimensions
  --agents N                  Agent count (default: 10)
  --steps N                   Fixed steps (default: 100)
  --seed N                    Random seed (default: 42)
  --spawn-radius R            Initial disk radius (default: 3)
  --dt T                      Fixed time step (default: 0.1)
  --speed V                   Maximum copter speed / Boids speed (default: 1)
  --max-acceleration A        Copter acceleration limit (default: 1)
  --max-braking B             Copter braking limit (default: 2)
  --max-jerk J                Copter acceleration-change limit (default: 8)
  --turn-response M           Free Boids heading response in [0, 1] (default: 0.1)
  --neighbor-radius R         Omnidirectional perception radius (default: 1)
  --separation-radius R       Separation activation radius (default: 0.3)
  --body-radius R             Physical body radius (default: 0.025)
  --separation-weight W        Nonnegative rule weight (default: 10)
  --alignment-weight W         Nonnegative rule weight (default: 1)
  --cohesion-weight W          Nonnegative rule weight (default: 3)
  --command-weight W           Nonnegative collective command weight (default: 1)
  --direction X Y              Steer the flock along a direction
  --gather X Y R               Gather near the center; overflow outside is allowed
  --spread GAP                 Increase neighbor spacing (body margin <= GAP <= view)
  --hold                      Hover at current positions
  --square X Y SIDE           One static square obstacle
  --square-angle DEG          Obstacle rotation, counterclockwise (default: 0)
  --wind X Y SIDE VX VY       Add a square wind-velocity field (repeatable)
  --wind-rect X Y W L DEG V   Rectangular wind field; V is nonnegative speed
  --noise X Y SIDE SIGMA      Add Gaussian position-measurement noise
  --noise-rect X Y W L DEG S  Rectangular noise field; S is position sigma
  --waypoint X Y              Append a route point (repeatable)
  --route-width W             Full corridor width (default: 2)
  --route-centering W          Soft center bias (default: 0.25; 0 disables)
  --route-cohesion W           Ease off for followers (default: 1; 0 disables)
  --route-radius R            Alias for corridor half-width
  --obstacle-clearance R      Extra planning clearance (default: 0.15)
  --obstacle-weight W         Nonnegative obstacle avoidance cost (default: 1)
  --trace PATH                Create a new JSONL file with every state snapshot
  --help                      Show this help

Without a command, use free flocking. Choose at most one command.
";

fn value<T: FromStr>(
    args: &mut impl Iterator<Item = String>,
    flag: &str,
) -> Result<T, Box<dyn Error>> {
    args.next()
        .ok_or_else(|| format!("missing value for {flag}"))?
        .parse()
        .map_err(|_| format!("invalid value for {flag}").into())
}

pub(super) struct GroupOptions {
    pub config: Config,
    pub spawn: SpawnConfig,
    pub command: Command,
    pub route: Option<Route>,
}
fn finish_group(
    groups: &mut Vec<GroupOptions>,
    config: Config,
    spawn: SpawnConfig,
    mode: Command,
    command: Option<Command>,
    points: Vec<[f64; 2]>,
    radius: Option<f64>,
) -> Result<(), Box<dyn Error>> {
    if !points.is_empty() && command.is_some() || points.is_empty() && radius.is_some() {
        return Err("route points require their own command".into());
    }
    let route = if points.is_empty() {
        None
    } else {
        Some(Route::new(points, radius.unwrap_or(1.0))?)
    };
    groups.push(GroupOptions {
        config,
        spawn,
        command: command.unwrap_or(mode),
        route,
    });
    Ok(())
}
pub(super) struct Options {
    pub groups: Vec<GroupOptions>,
    pub interactions: Vec<(usize, usize, Interaction)>,
    pub config: Config,
    pub count: usize,
    pub steps: u64,
    pub seed: u64,
    pub spawn_radius: f64,
    pub command: Option<Command>,
    pub obstacle: Option<Rectangle>,
    pub winds: Vec<WindZone>,
    pub noises: Vec<NoiseZone>,
    pub waypoints: Vec<[f64; 2]>,
    pub route_radius: Option<f64>,
    pub trace_path: Option<String>,
}

impl Options {
    pub fn parse(mut args: impl Iterator<Item = String>) -> Result<Option<Self>, Box<dyn Error>> {
        let mut config = Config::default();
        let mut groups = Vec::new();
        let mut current_group = None;
        let mut interactions = Vec::new();
        let mut explicit_count = false;
        let (mut count, mut steps, mut seed, mut spawn_radius) = (10_usize, 100_u64, 42_u64, 3.0);
        let mut command = None;
        let mut obstacle = None;
        let mut obstacle_angle: Option<f64> = None;
        let mut winds = Vec::new();
        let mut noises = Vec::new();
        let mut waypoints = Vec::new();
        let mut route_radius = None;
        let mut trace_path: Option<String> = None;
        while let Some(flag) = args.next() {
            match flag.as_str() {
                "--help" => {
                    return Ok(None);
                }
                "--empty" => count = 0,
                "--agents" => {
                    count = value(&mut args, &flag)?;
                    explicit_count = true;
                }
                "--group" => {
                    if let Some((spawn, mode)) = current_group.take() {
                        finish_group(
                            &mut groups,
                            config,
                            spawn,
                            mode,
                            command.take(),
                            std::mem::take(&mut waypoints),
                            route_radius.take(),
                        )?;
                    } else if command.is_some() || !waypoints.is_empty() {
                        return Err("declare a group before its command / route".into());
                    }
                    let center = [value(&mut args, &flag)?, value(&mut args, &flag)?];
                    let count = value(&mut args, &flag)?;
                    let radius = value(&mut args, &flag)?;
                    let seed = value(&mut args, &flag)?;
                    let mode: String = value(&mut args, &flag)?;
                    let mode = match mode.as_str() {
                        "boids" => Command::Flock,
                        "copters" => Command::Hold,
                        _ => return Err("group mode must be copters or boids".into()),
                    };
                    current_group = Some((
                        SpawnConfig {
                            count,
                            center,
                            radius,
                            seed,
                        },
                        mode,
                    ));
                }
                "--interaction" => {
                    let source = value(&mut args, &flag)?;
                    let target = value(&mut args, &flag)?;
                    let mode: String = value(&mut args, &flag)?;
                    let mode = match mode.as_str() {
                        "react" => Interaction::React,
                        "avoid" => Interaction::Avoid,
                        "ignore" => Interaction::Ignore,
                        _ => return Err("interaction must be react, avoid or ignore".into()),
                    };
                    interactions.push((source, target, mode));
                }
                "--steps" => steps = value(&mut args, &flag)?,
                "--seed" => seed = value(&mut args, &flag)?,
                "--spawn-radius" => spawn_radius = value(&mut args, &flag)?,
                "--dt" => config.dt = value(&mut args, &flag)?,
                "--speed" => config.speed = value(&mut args, &flag)?,
                "--max-acceleration" => config.max_acceleration = value(&mut args, &flag)?,
                "--max-braking" => config.max_braking = value(&mut args, &flag)?,
                "--max-jerk" => config.max_jerk = value(&mut args, &flag)?,
                "--turn-response" => config.turn_response = value(&mut args, &flag)?,
                "--neighbor-radius" => config.neighbor_radius = value(&mut args, &flag)?,
                "--separation-radius" => config.separation_radius = value(&mut args, &flag)?,
                "--body-radius" => config.body_radius = value(&mut args, &flag)?,
                "--separation-weight" => config.separation_weight = value(&mut args, &flag)?,
                "--alignment-weight" => config.alignment_weight = value(&mut args, &flag)?,
                "--cohesion-weight" => config.cohesion_weight = value(&mut args, &flag)?,
                "--command-weight" => config.command_weight = value(&mut args, &flag)?,
                "--obstacle-clearance" => config.obstacle_clearance = value(&mut args, &flag)?,
                "--route-centering" => config.route_centering = value(&mut args, &flag)?,
                "--route-cohesion" => config.route_cohesion = value(&mut args, &flag)?,
                "--spread" => {
                    if command.is_some() {
                        return Err("choose only one collective command".into());
                    }
                    command = Some(Command::Spread {
                        spacing: value(&mut args, &flag)?,
                    });
                }
                "--obstacle-weight" => config.obstacle_weight = value(&mut args, &flag)?,
                "--square" | "--rectangle" => {
                    if obstacle.is_some() {
                        return Err("only one solid obstacle is supported".into());
                    }
                    let center = [value(&mut args, &flag)?, value(&mut args, &flag)?];
                    let width = value(&mut args, &flag)?;
                    let height = if flag == "--square" {
                        width
                    } else {
                        value(&mut args, &flag)?
                    };
                    obstacle = Some(Rectangle::new(center, height, width)?);
                }
                "--square-angle" | "--obstacle-angle" => {
                    if obstacle_angle.is_some() {
                        return Err("choose only one square angle".into());
                    }
                    obstacle_angle = Some(value(&mut args, &flag)?);
                }
                "--wind-rect" | "--noise-rect" => {
                    let center = [value(&mut args, &flag)?, value(&mut args, &flag)?];
                    let width = value(&mut args, &flag)?;
                    let length = value(&mut args, &flag)?;
                    let angle: f64 = value(&mut args, &flag)?;
                    let strength = value(&mut args, &flag)?;
                    if flag == "--wind-rect" {
                        winds.push(WindZone::rectangular(
                            center,
                            width,
                            length,
                            angle.to_radians(),
                            strength,
                        )?);
                    } else {
                        noises.push(NoiseZone::new(
                            Rectangle::new(center, width, length)?.rotated(angle.to_radians())?,
                            strength,
                        )?);
                    }
                }
                "--wind" | "--noise" => {
                    let area = Square::new(
                        [value(&mut args, &flag)?, value(&mut args, &flag)?],
                        value(&mut args, &flag)?,
                    )?;
                    if flag == "--wind" {
                        winds.push(WindZone::new(
                            area,
                            [value(&mut args, &flag)?, value(&mut args, &flag)?],
                        )?);
                    } else {
                        noises.push(NoiseZone::new(area, value(&mut args, &flag)?)?);
                    }
                }
                "--waypoint" => {
                    waypoints.push([value(&mut args, &flag)?, value(&mut args, &flag)?])
                }
                "--route-radius" | "--route-width" => {
                    if route_radius.is_some() {
                        return Err("choose only one route width".into());
                    }
                    let size: f64 = value(&mut args, &flag)?;
                    route_radius = Some(if flag == "--route-width" {
                        size * 0.5
                    } else {
                        size
                    });
                }
                "--trace" => {
                    if trace_path.is_some() {
                        return Err("choose only one trace file".into());
                    }
                    trace_path = Some(value(&mut args, &flag)?);
                }
                "--hold" => {
                    if command.is_some() {
                        return Err("choose only one collective command".into());
                    }
                    command = Some(Command::Hold);
                }
                "--direction" | "--gather" => {
                    if command.is_some() {
                        return Err("choose only one collective command".into());
                    }
                    let point = [value(&mut args, &flag)?, value(&mut args, &flag)?];
                    command = Some(if flag == "--direction" {
                        Command::Move(point)
                    } else {
                        Command::Gather {
                            center: point,
                            radius: value(&mut args, &flag)?,
                        }
                    });
                }
                _ => return Err(format!("unknown option: {flag}; use --help").into()),
            }
        }

        if let Some((spawn, mode)) = current_group {
            finish_group(
                &mut groups,
                config,
                spawn,
                mode,
                command.take(),
                std::mem::take(&mut waypoints),
                route_radius.take(),
            )?;
        }
        if !groups.is_empty() && explicit_count {
            return Err("use --agents for a legacy run or --group for independent groups".into());
        }
        if (!waypoints.is_empty() && command.is_some())
            || (waypoints.is_empty() && route_radius.is_some())
        {
            return Err("route points require their own command".into());
        }
        if obstacle_angle.is_some() && obstacle.is_none() {
            return Err("square angle requires a square".into());
        }
        let obstacle = obstacle
            .map(|square| square.rotated(obstacle_angle.unwrap_or(0.0).to_radians()))
            .transpose()?;
        Ok(Some(Self {
            groups,
            interactions,
            config,
            count,
            steps,
            seed,
            spawn_radius,
            command,
            obstacle,
            winds,
            noises,
            waypoints,
            route_radius,
            trace_path,
        }))
    }
}
