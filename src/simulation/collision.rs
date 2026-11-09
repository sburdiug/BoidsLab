use super::{
    Agent, AgentState, Command, Config, Simulation, SimulationError, math::*, spatial::SpatialIndex,
};

#[derive(Clone, Copy)]
enum Event {
    Contact(usize, usize),
    Arrival(usize),
    Obstacle(usize),
}

pub(super) fn resolve_motion(
    world: &Simulation,
    agents: &mut [Agent],
) -> Result<(), SimulationError> {
    let config = world.config();
    // A body can travel at most speed * dt; this broad phase includes swept contacts.
    let ground_speed = agents.iter().map(|a| norm(a.velocity())).fold(
        world
            .groups()
            .iter()
            .map(|g| g.config().speed)
            .fold(config.speed, f64::max),
        f64::max,
    );
    if !ground_speed.is_finite() {
        return Err(SimulationError::NumericOverflow);
    }
    let body = world
        .groups()
        .iter()
        .map(|g| g.config().body_radius)
        .fold(config.body_radius, f64::max);
    let reach = 2.0 * body + 2.0 * ground_speed * config.dt;
    let grid = SpatialIndex::new(
        agents,
        if (1e-75..1e75).contains(&ground_speed) {
            reach
        } else {
            f64::NAN
        },
    );
    let mut pairs = Vec::new();
    for (i, agent) in agents.iter().enumerate() {
        if agent.state == AgentState::Terminated {
            continue;
        }
        for j in grid.candidates(agent.position, agents) {
            if j > i {
                pairs.push((i, j));
            }
        }
    }
    resolve_pairs(world, agents, &pairs)
}

fn resolve_pairs(
    world: &Simulation,
    agents: &mut [Agent],
    pairs: &[(usize, usize)],
) -> Result<(), SimulationError> {
    let dt = world.config().dt;
    let mut remaining = dt;
    while remaining > 0.0 {
        let mut events = Vec::new();
        for (i, agent) in agents.iter().enumerate() {
            if agent.state == AgentState::Terminated {
                continue;
            }
            let config = world.agent_config(i);
            let group = world.agent_group(i);
            let command = group.command();
            if agent.state == AgentState::Moving
                && norm(agent.drift) <= config.speed
                && agent.settled_time >= 0.3
                && norm(agent.velocity()) <= config.max_braking * config.dt
                && norm(agent.acceleration) <= config.max_jerk * config.dt
                && match command {
                    Command::Gather { center, radius } => {
                        near_region(i, agents, center, radius, config)
                    }
                    Command::Spread { spacing } => agents.iter().enumerate().all(|(j, other)| {
                        j == i
                            || !world.reacts_to(i, j)
                            || other.state == AgentState::Terminated
                            || norm(sub(agent.position, other.position)) > config.neighbor_radius
                            || norm(sub(agent.position, other.position)) >= spacing * 0.98
                    }),
                    Command::FollowRoute => group.route().is_some_and(|r| {
                        agent.waypoint_index + 1 == r.points().len()
                            && near_region(
                                i,
                                agents,
                                *r.points().last().unwrap(),
                                r.radius(),
                                config,
                            )
                    }),
                    Command::Hold => true,
                    _ => false,
                }
                && agents.iter().enumerate().all(|(j, other)| {
                    j == i
                        || group.interaction(other.group_id) == super::Interaction::Ignore
                        || other.state == AgentState::Terminated
                        || norm(sub(agent.position, other.position)) > config.neighbor_radius
                        || norm(sub(agent.position, other.position))
                            > 1.2 * (config.body_radius + world.agent_config(j).body_radius)
                })
            {
                events.push((0.0, Event::Arrival(i)));
            }
            if let Some(square) = world.obstacle()
                && let Some(time) = square.contact(
                    agent.position,
                    agent.velocity(),
                    config.body_radius,
                    remaining,
                )?
            {
                events.push((time, Event::Obstacle(i)));
            }
        }
        for &(i, j) in pairs {
            let (agent, other) = (&agents[i], &agents[j]);
            if agent.state == AgentState::Terminated || other.state == AgentState::Terminated {
                continue;
            }
            if let Some(t) = contact_time(
                sub(agent.position, other.position),
                sub(agent.velocity(), other.velocity()),
                world.agent_config(i).body_radius + world.agent_config(j).body_radius,
                remaining,
            )? {
                events.push((t, Event::Contact(i, j)));
            }
        }
        let earliest = events.iter().map(|(t, _)| *t).reduce(f64::min);
        let duration = earliest.unwrap_or(remaining);
        for agent in agents.iter_mut() {
            if agent.state != AgentState::Terminated {
                let position = add(agent.position, scale(agent.velocity(), duration));
                if !finite(position) {
                    return Err(SimulationError::NumericOverflow);
                }
                agent.position = position;
            }
        }
        let Some(earliest) = earliest else {
            break;
        };
        // Events within this time tolerance are treated as simultaneous.
        events.retain(|(t, _)| (*t - earliest).abs() <= 1e-10 * dt.min(1.0));
        for (_, event) in &events {
            if let Event::Contact(i, j) = *event {
                for index in [i, j] {
                    agents[index].state = AgentState::Terminated;
                    agents[index].velocity = [0.0; 2];
                    agents[index].acceleration = [0.0; 2];
                    agents[index].drift = [0.0; 2];
                }
            }
            if let Event::Obstacle(index) = *event {
                agents[index].state = AgentState::Terminated;
                agents[index].velocity = [0.0; 2];
                agents[index].acceleration = [0.0; 2];
                agents[index].drift = [0.0; 2];
            }
        }
        for (_, event) in events {
            if let Event::Arrival(i) = event
                && agents[i].state == AgentState::Moving
            {
                agents[i].state = AgentState::Arrived;
                agents[i].velocity =
                    limited(scale(agents[i].drift, -1.0), world.agent_config(i).speed);
                agents[i].acceleration = [0.0; 2];
                agents[i].hold_position = agents[i].position;
            }
        }
        remaining = (remaining - duration).max(0.0);
    }
    Ok(())
}

fn near_region(
    index: usize,
    agents: &[Agent],
    center: [f64; 2],
    radius: f64,
    config: Config,
) -> bool {
    let agent = agents[index];
    let distance = norm(sub(agent.position, center));
    distance <= radius + config.body_radius
        || agents.iter().enumerate().any(|(j, other)| {
            j != index
                && other.group_id == agent.group_id
                && other.state == AgentState::Arrived
                && norm(sub(agent.position, other.position)) <= config.neighbor_radius
                && norm(sub(other.position, center)) < distance
        })
}

pub(super) fn contact_time(
    position: [f64; 2],
    velocity: [f64; 2],
    radius: f64,
    horizon: f64,
) -> Result<Option<f64>, SimulationError> {
    let c = dot(position, position) - radius * radius;
    let a = dot(velocity, velocity);
    let b = dot(position, velocity);
    if ![a, b, c].iter().all(|v| v.is_finite()) {
        return Err(SimulationError::NumericOverflow);
    }
    if c <= 0.0 {
        return Ok(Some(0.0));
    }
    if a == 0.0 && velocity != [0.0; 2] {
        return Err(SimulationError::NumericOverflow);
    }
    if a == 0.0 || b >= 0.0 {
        return Ok(None);
    }
    let discriminant = b * b - a * c;
    if !discriminant.is_finite() {
        return Err(SimulationError::NumericOverflow);
    }
    if discriminant < 0.0 {
        return Ok(None);
    }
    let time = c / (-b + discriminant.sqrt());
    Ok((time <= horizon).then_some(time))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::{Config, Simulation};

    #[test]
    fn swept_grid_resolution_matches_all_pairs_with_arrivals_and_multiple_contacts() {
        for dt in [0.01, 0.1, 1.0, 10.0] {
            for seed in [0, 42, 99] {
                let config = Config {
                    dt,
                    ..Config::default()
                };
                let mut indexed = Simulation::seeded(config, 60, 3.0, seed).unwrap().agents;
                for (i, agent) in indexed.iter_mut().enumerate() {
                    if i % 5 == 0 {
                        agent.state = AgentState::Arrived;
                        agent.velocity = [0.0; 2];
                    } else if i % 7 == 0 {
                        agent.velocity = scale(agent.velocity, 0.0001);
                        agent.settled_time = 0.3;
                    }
                }
                let mut brute = indexed.clone();
                let pairs: Vec<_> = (0..brute.len())
                    .flat_map(|i| (i + 1..brute.len()).map(move |j| (i, j)))
                    .collect();
                let command = Command::Gather {
                    center: [0.0; 2],
                    radius: 3.0,
                };
                for _ in 0..6 {
                    let mut world = Simulation::seeded(config, 60, 3.0, seed).unwrap();
                    world.groups[0].command = command;
                    resolve_motion(&world, &mut indexed).unwrap();
                    resolve_pairs(&world, &mut brute, &pairs).unwrap();
                    assert_eq!(indexed, brute, "dt={dt}, seed={seed}");
                }
            }
        }
    }
}
