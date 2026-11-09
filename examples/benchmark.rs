//! Reproducible core timing; excludes compilation, rendering and summary output.
use rust_playground::simulation::{AgentState, Command, Config, Simulation};
use std::{error::Error, time::Instant};

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 4 {
        return Err("usage: benchmark AGENTS STEPS SPAWN_RADIUS gather|cluster|flock|hold".into());
    }
    let count = args[0].parse()?;
    let steps: usize = args[1].parse()?;
    if steps == 0 {
        return Err("steps must be positive".into());
    }
    let radius = args[2].parse()?;
    let command = match args[3].as_str() {
        "gather" => Command::Gather {
            center: [10.0, 0.0],
            radius: 3.0,
        },
        "cluster" => Command::Gather {
            center: [0.0, 0.0],
            radius: 1.0,
        },
        "flock" => Command::Flock,
        "hold" => Command::Hold,
        _ => return Err("unknown command".into()),
    };
    let start = Instant::now();
    let mut world = Simulation::seeded(Config::default(), count, radius, 42)?;
    if matches!(command, Command::Gather { .. }) {
        world.set_command(Command::Hold)?;
    }
    world.set_command(command)?;
    let init_ms = start.elapsed().as_secs_f64() * 1_000.0;
    let mut times = Vec::with_capacity(steps);
    for _ in 0..steps {
        let start = Instant::now();
        world.step()?;
        times.push(start.elapsed().as_secs_f64() * 1_000.0);
    }
    let total: f64 = times.iter().sum();
    times.sort_by(f64::total_cmp);
    let p95 = times[((steps as f64 * 0.95).ceil() as usize - 1).min(steps - 1)];
    let mut hash = 0xcbf29ce484222325_u64;
    let mut states = [0; 3];
    for agent in world.agents() {
        let state = match agent.state() {
            AgentState::Moving => 0,
            AgentState::Arrived => 1,
            AgentState::Terminated => 2,
        };
        states[state] += 1;
        for value in agent
            .position()
            .into_iter()
            .chain(agent.velocity())
            .chain(agent.acceleration())
        {
            hash = (hash ^ value.to_bits()).wrapping_mul(0x100000001b3);
        }
        hash = (hash ^ state as u64).wrapping_mul(0x100000001b3);
    }
    println!(
        "agents,steps,radius,command,init_ms,total_ms,mean_ms,p95_ms,moving,arrived,terminated,state_hash"
    );
    println!(
        "{count},{steps},{radius},{},{init_ms:.3},{total:.3},{:.3},{p95:.3},{},{},{},{hash:016x}",
        args[3],
        total / steps as f64,
        states[0],
        states[1],
        states[2]
    );
    Ok(())
}
