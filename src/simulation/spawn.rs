use super::{
    Agent, AgentState, Command, Config, EventKind, Group, GroupId, Rectangle, Simulation,
    SimulationError, SpawnConfig, math::*,
};
use rand::{RngExt, SeedableRng, rngs::StdRng};
use std::f64::consts::TAU;

impl Simulation {
    /// Agent indices are stable identifiers: terminated agents are never removed.
    pub fn new(config: Config, mut agents: Vec<Agent>) -> Result<Self, SimulationError> {
        config.validate()?;
        if agents.is_empty() {
            return Err(SimulationError::InvalidInput(
                "at least one agent is required",
            ));
        }
        for i in 0..agents.len() {
            if agents[i].state != AgentState::Moving {
                return Err(SimulationError::InvalidInput(
                    "initial agents must be moving",
                ));
            }
            for other in &agents[..i] {
                if norm(sub(agents[i].position, other.position)) <= 2.0 * config.body_radius {
                    return Err(SimulationError::InvalidInput(
                        "initial agent bodies overlap or touch",
                    ));
                }
            }
            let direction = unit(agents[i].velocity).ok_or(SimulationError::InvalidInput(
                "initial velocity must be finite and nonzero",
            ))?;
            agents[i].velocity = scale(direction, config.speed);
            agents[i].settled_time = 0.0;
            agents[i].drift = [0.0; 2];
            agents[i].hold_position = agents[i].position;
            agents[i].waypoint_index = 0;
            agents[i].route_offset = 0.0;
        }
        let count = agents.len();
        let mut world = Self::empty(config)?;
        let spawn = super::SpawnConfig {
            count,
            center: [0.0; 2],
            radius: 3.0,
            seed: 0,
        };
        world.agents = agents;
        world.groups.push(super::Group::new(
            0,
            config,
            spawn,
            Command::Flock,
            0..count,
            false,
        ));
        world.record(super::EventKind::GroupAdded {
            seeded: false,
            group: 0,
            config,
            spawn,
            command: Command::Flock,
        });
        Ok(world)
    }

    /// Samples a uniform disk and headings using the pinned rand implementation.
    pub fn seeded(
        config: Config,
        count: usize,
        spawn_radius: f64,
        seed: u64,
    ) -> Result<Self, SimulationError> {
        Self::seeded_with_obstacle(config, count, spawn_radius, seed, None)
    }

    pub fn seeded_with_obstacle(
        config: Config,
        count: usize,
        spawn_radius: f64,
        seed: u64,
        obstacle: Option<Rectangle>,
    ) -> Result<Self, SimulationError> {
        let mut world = Self::empty(config)?;
        world.set_obstacle(obstacle)?;
        world.add_group(
            config,
            super::SpawnConfig {
                count,
                center: [0.0; 2],
                radius: spawn_radius,
                seed,
            },
            Command::Flock,
        )?;
        Ok(world)
    }
}

impl Simulation {
    pub fn add_group(
        &mut self,
        config: Config,
        spawn: SpawnConfig,
        command: Command,
    ) -> Result<GroupId, SimulationError> {
        config.validate()?;
        if config.dt != self.config.dt {
            return Err(SimulationError::InvalidInput(
                "all groups must share the scene time step",
            ));
        }
        if spawn.count == 0
            || !finite(spawn.center)
            || !spawn.radius.is_finite()
            || spawn.radius <= 0.0
        {
            return Err(SimulationError::InvalidInput(
                "spawn count/radius must be positive and center finite",
            ));
        }
        let mut rng = StdRng::seed_from_u64(spawn.seed);
        let mut pending: Vec<Agent> = Vec::new();
        pending
            .try_reserve(spawn.count)
            .map_err(|_| SimulationError::InvalidInput("agent count cannot be allocated"))?;
        for _ in 0..spawn.count {
            let mut placed = false;
            for _ in 0..1000 {
                let angle = rng.random_range(0.0..TAU);
                let radius = spawn.radius * rng.random::<f64>().sqrt();
                let p = add(spawn.center, [radius * angle.cos(), radius * angle.sin()]);
                if !finite(p) {
                    return Err(SimulationError::NumericOverflow);
                }
                if self
                    .obstacles
                    .iter()
                    .all(|o| o.distance(p) > config.body_radius)
                    && pending
                        .iter()
                        .all(|a| norm(sub(p, a.position)) > 2.0 * config.body_radius)
                    && self.agents.iter().enumerate().all(|(i, a)| {
                        a.state == AgentState::Terminated
                            || norm(sub(p, a.position))
                                > config.body_radius + self.agent_config(i).body_radius
                    })
                {
                    let heading = rng.random_range(0.0..TAU);
                    pending.push(Agent::new(p, [heading.cos(), heading.sin()])?);
                    placed = true;
                    break;
                }
            }
            if !placed {
                return Err(SimulationError::PlacementFailed);
            }
        }
        let mut validated = Self::new(config, pending)?;
        validated.obstacles = self.obstacles.clone();
        validated.set_command(command)?;
        let id = self.groups.len();
        let start = self.agents.len();
        self.agents
            .try_reserve(spawn.count)
            .map_err(|_| SimulationError::InvalidInput("agent count cannot be allocated"))?;
        self.groups
            .try_reserve(1)
            .map_err(|_| SimulationError::InvalidInput("group cannot be allocated"))?;
        for a in &mut validated.agents {
            a.group_id = id;
        }
        self.agents.extend(validated.agents);
        let command = validated.groups[0].command;
        self.groups.push(Group::new(
            id,
            config,
            spawn,
            command,
            start..self.agents.len(),
            true,
        ));
        self.record(EventKind::GroupAdded {
            seeded: true,
            group: id,
            config,
            spawn,
            command,
        });
        Ok(id)
    }
}
