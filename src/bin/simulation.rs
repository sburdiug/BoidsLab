use rust_playground::simulation::{Command, Route, ScenePreset, Simulation};
use std::{
    env,
    error::Error,
    fs::OpenOptions,
    io::{self, Write},
    process::ExitCode,
};

#[path = "simulation/options.rs"]
mod options;
#[path = "simulation/output.rs"]
mod output;
use options::{HELP, Options};
use output::write_snapshot;

fn run() -> Result<(), Box<dyn Error>> {
    let Some(Options {
        groups,
        interactions,
        config,
        count,
        steps,
        seed,
        spawn_radius,
        command,
        obstacles,
        winds,
        noises,
        waypoints,
        route_radius,
        trace_path,
        scene_path,
    }) = Options::parse(env::args().skip(1))?
    else {
        io::stdout().write_all(HELP.as_bytes())?;
        return Ok(());
    };
    let preset = scene_path.map(ScenePreset::load).transpose()?;
    let mut world = if groups.is_empty() {
        let mut world = Simulation::empty(config)?;
        if let Some(preset) = &preset {
            preset.apply(&mut world)?;
        }
        for &shape in &obstacles {
            world.add_obstacle(shape)?;
        }
        if count > 0 {
            world.add_group(
                config,
                rust_playground::simulation::SpawnConfig {
                    count,
                    center: [0.0; 2],
                    radius: spawn_radius,
                    seed,
                },
                Command::Flock,
            )?;
        }
        for wind in winds {
            world.add_wind(wind);
        }
        for noise in noises {
            world.add_noise(noise);
        }
        if matches!(
            command,
            Some(Command::Gather { .. } | Command::Spread { .. })
        ) {
            world.set_command(Command::Hold)?;
        }
        world.set_command(command.unwrap_or(if count == 0 {
            Command::Hold
        } else {
            Command::Flock
        }))?;
        if !waypoints.is_empty() {
            world.set_command(Command::Hold)?;
            let route = Route::new(waypoints, route_radius.unwrap_or(1.0))?;
            if count == 0 {
                world.add_path(route);
            } else {
                world.set_route(route)?;
            }
        }
        world
    } else {
        let mut world = Simulation::empty(groups[0].config)?;
        if let Some(preset) = &preset {
            preset.apply(&mut world)?;
        }
        for &shape in &obstacles {
            world.add_obstacle(shape)?;
        }
        for wind in winds {
            world.add_wind(wind);
        }
        for noise in noises {
            world.add_noise(noise);
        }
        for group in groups {
            let command = if group.route.is_some() {
                Command::Hold
            } else {
                group.command
            };
            let id = world.add_group(group.config, group.spawn, command)?;
            if let Some(route) = group.route {
                let path = world.add_path(route);
                world.assign_path(id, path)?;
            }
        }
        world
    };
    for (source, target, mode) in interactions {
        world.set_interaction(source, target, mode)?;
    }
    let mut trace = trace_path
        .map(|path| {
            OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(path)
                .map(io::BufWriter::new)
        })
        .transpose()?;
    let mut event_cursor = 0;
    if let Some(out) = &mut trace {
        write_snapshot(out, &world, seed, spawn_radius, "snapshot", event_cursor)?;
        event_cursor = world.events().len();
    }
    for _ in 0..steps {
        world.step()?;
        if let Some(out) = &mut trace {
            write_snapshot(
                out,
                &world,
                seed,
                spawn_radius,
                if world.steps() == steps {
                    "horizon"
                } else {
                    "snapshot"
                },
                event_cursor,
            )?;
            event_cursor = world.events().len();
        }
    }
    if let Some(out) = &mut trace {
        out.flush()?;
    }
    let mut out = io::BufWriter::new(io::stdout().lock());
    write_snapshot(&mut out, &world, seed, spawn_radius, "horizon", 0)?;
    out.flush()?;
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("simulation: {error}");
            ExitCode::FAILURE
        }
    }
}
