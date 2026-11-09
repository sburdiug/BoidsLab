use super::{
    AgentState, Command, Config, EventKind, GroupId, Simulation, SimulationError, math::*,
};

impl Config {
    /// FNV-1a over ordered little-endian behavior fields; global dt and spawn settings are excluded.
    pub fn profile_hash(self) -> u64 {
        let values = [
            self.speed,
            self.turn_response,
            self.neighbor_radius,
            self.separation_radius,
            self.body_radius,
            self.separation_weight,
            self.alignment_weight,
            self.cohesion_weight,
            self.command_weight,
            self.obstacle_clearance,
            self.obstacle_weight,
            self.route_centering,
            self.route_cohesion,
        ];
        let mut hash = 0xcbf29ce484222325_u64;
        for byte in b"boids-profile-v2".iter().copied().chain(
            values
                .into_iter()
                .flat_map(|v| if v == 0.0 { 0_u64 } else { v.to_bits() }.to_le_bytes()),
        ) {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
        hash
    }
}

impl Simulation {
    pub fn set_group_config(&mut self, id: GroupId, config: Config) -> Result<(), SimulationError> {
        config.validate()?;
        if config.dt != self.config.dt {
            return Err(SimulationError::InvalidInput(
                "time step belongs to the scene",
            ));
        }
        let group = self
            .group(id)
            .ok_or(SimulationError::InvalidInput("unknown group"))?;
        self.check_command(id, group.command, config)?;
        if let Some(route) = &group.route {
            for &point in route.points() {
                self.validate_group_target(id, point, config.body_radius)?;
            }
        }
        for i in group.agents.clone() {
            let a = self.agents[i];
            if a.state == AgentState::Terminated {
                continue;
            }
            if self
                .obstacles
                .iter()
                .any(|o| o.distance(a.position) <= config.body_radius)
                || self.agents.iter().enumerate().any(|(j, b)| {
                    j != i
                        && b.state != AgentState::Terminated
                        && norm(sub(a.position, b.position))
                            <= config.body_radius
                                + if b.group_id == id {
                                    config.body_radius
                                } else {
                                    self.agent_config(j).body_radius
                                }
                })
            {
                return Err(SimulationError::InvalidInput(
                    "new body radius overlaps a live body or obstacle",
                ));
            }
        }
        let group = &mut self.groups[id];
        group.config = config;
        group.profile_hash = config.profile_hash();
        for a in &mut self.agents[group.agents.clone()] {
            if a.state != AgentState::Terminated {
                a.local_target = None;
                a.velocity = if matches!(group.command, Command::Flock | Command::Move(_)) {
                    scale(unit(a.velocity).unwrap_or([1.0, 0.0]), config.speed)
                } else {
                    limited(a.velocity, config.speed)
                };
                a.settled_time = 0.0;
            }
        }
        if id == 0 {
            self.config = config;
        }
        self.record(EventKind::ProfileChanged { group: id, config });
        Ok(())
    }
    pub fn restarted(&self) -> Result<Self, SimulationError> {
        // shortcut: explicit-agent groups have no seed recipe; add snapshot loading before supporting their restart.
        if self.groups.iter().any(|g| !g.seeded) {
            return Err(SimulationError::InvalidInput(
                "only seeded groups can be regenerated",
            ));
        }
        let mut world = Self::empty(self.config)?;
        world.paths = self.paths.clone();
        world.obstacles = self.obstacles.clone();
        world.wind_zones = self.wind_zones.clone();
        world.noise_zones = self.noise_zones.clone();
        for group in &self.groups {
            let command = if group.command == Command::FollowRoute {
                Command::Hold
            } else {
                group.command
            };
            let id = world.add_group(group.config, group.spawn, command)?;
            if let Some(route) = &group.route {
                world.set_group_route(id, route.clone())?;
            }
        }
        for group in &self.groups {
            for &(target, mode) in &group.interactions {
                world.set_interaction(group.id, target, mode)?;
            }
        }
        Ok(world)
    }
}
