use super::{
    AgentState, Command, Simulation, SimulationError, collision::resolve_motion, math::*,
    spatial::SpatialIndex,
};

impl Simulation {
    /// A failed step leaves the entire previous state and clock unchanged.
    pub fn step(&mut self) -> Result<(), SimulationError> {
        let steps = self
            .steps
            .checked_add(1)
            .ok_or(SimulationError::NumericOverflow)?;
        if !(steps as f64 * self.config.dt).is_finite() {
            return Err(SimulationError::NumericOverflow);
        }
        let mut next = self.agents.clone();
        let start_time = self.time();
        let mut contacts = Vec::new();
        if self.groups.iter().any(|g| g.command() != Command::Hold)
            || !self.wind_zones.is_empty()
            || !self.noise_zones.is_empty()
            || self
                .agents
                .iter()
                .any(|a| a.velocity != [0.0; 2] || a.drift != [0.0; 2])
        {
            for agent in &mut next {
                if agent.state != AgentState::Terminated {
                    agent.drift = self.wind_at(agent.position)?;
                }
            }
            let mut observed = next.clone();
            for (i, agent) in observed.iter_mut().enumerate() {
                if agent.state == AgentState::Terminated {
                    continue;
                }
                agent.position = self.measured_position(i)?;
                if let Some(route) = self.agent_group(i).route() {
                    while agent.waypoint_index + 1 < route.points().len()
                        && route.reached(agent.waypoint_index, agent.position)
                    {
                        agent.route_offset =
                            route.lateral_offset(agent.waypoint_index, agent.position);
                        agent.waypoint_index += 1;
                    }
                    next[i].waypoint_index = agent.waypoint_index;
                    next[i].route_offset = agent.route_offset;
                }
            }
            let view = self
                .groups
                .iter()
                .map(|g| g.config().neighbor_radius)
                .fold(self.config.neighbor_radius, f64::max);
            let grid = SpatialIndex::new(&observed, view);
            let mut neighbors = Vec::new();
            for (i, agent) in next.iter_mut().enumerate() {
                if agent.state != AgentState::Terminated {
                    let config = self.agent_config(i);
                    let command = self.agent_group(i).command();
                    let region = matches!(
                        command,
                        Command::Gather { .. }
                            | Command::Spread { .. }
                            | Command::FollowRoute
                            | Command::Hold
                    );
                    let resting = agent.drift == [0.0; 2]
                        && agent.velocity == [0.0; 2]
                        && self.noise_zones.is_empty();
                    if agent.state == AgentState::Arrived
                        && resting
                        && !matches!(command, Command::Spread { .. })
                    {
                        continue;
                    }
                    self.visible_neighbors(i, &observed, &grid, &mut neighbors);
                    if let Command::Spread { spacing } = command
                        && neighbors.iter().any(|&j| {
                            self.reacts_to(i, j)
                                && norm(sub(observed[i].position, observed[j].position))
                                    < spacing * 0.98
                        })
                    {
                        agent.state = AgentState::Moving;
                    }
                    let hovering = agent.state == AgentState::Arrived;
                    if hovering && norm(agent.drift) > config.speed {
                        agent.state = AgentState::Moving;
                    }
                    if hovering && resting {
                        continue;
                    }
                    if region {
                        let goal = if hovering || command == Command::Hold {
                            agent.hold_position
                        } else if let Command::Spread { spacing } = command {
                            self.dispersion_goal(i, &neighbors, &observed, spacing)?
                        } else if command == Command::FollowRoute {
                            self.route_goal(i, &neighbors, &observed)?
                        } else {
                            let cached = agent.local_target.filter(|target| {
                                neighbors.iter().all(|&j| {
                                    norm(sub(*target, observed[j].position))
                                        > 1.1
                                            * (config.body_radius
                                                + self.agent_config(j).body_radius)
                                }) && !self.parking_path_blocked(i, *target, &neighbors, &observed)
                                    && self
                                        .obstacles
                                        .iter()
                                        .all(|o| o.distance(*target) > config.body_radius)
                            });
                            let target = match cached {
                                Some(target) => target,
                                None => self.local_goal(i, &neighbors, &observed)?,
                            };
                            agent.local_target = Some(target);
                            target
                        };
                        let mut velocity = self.steer(i, &neighbors, &observed, Some(goal))?;
                        if command == Command::FollowRoute {
                            velocity =
                                scale(velocity, self.route_speed_factor(i, &neighbors, &observed));
                        }
                        agent.velocity = velocity;
                        agent.settled_time = if norm(agent.velocity()) <= 0.001 * config.speed {
                            agent.settled_time + config.dt
                        } else {
                            0.0
                        };
                        if hovering
                            && norm(sub(agent.position, agent.hold_position))
                                > config.body_radius.max(0.05)
                        {
                            agent.state = AgentState::Moving;
                        }
                    } else {
                        agent.velocity = self.steer(i, &neighbors, &observed, None)?;
                    }
                }
            }
            self.reconcile_controlled_requests(&observed, &mut next, &grid)?;
            contacts = resolve_motion(self, &mut next)?;
        }
        let changes: Vec<_> = self
            .agents
            .iter()
            .zip(&next)
            .enumerate()
            .filter(|(_, (a, b))| a.state != b.state)
            .map(|(agent, (a, b))| super::EventKind::StateChanged {
                agent,
                from: a.state,
                to: b.state,
            })
            .collect();
        self.agents = next;
        self.steps = steps;
        for (offset, event) in contacts {
            self.record_at(event, start_time + offset);
        }
        for change in changes {
            self.record(change);
        }
        Ok(())
    }
}

impl Simulation {
    fn visible_neighbors(
        &self,
        index: usize,
        observed: &[super::Agent],
        grid: &SpatialIndex,
        neighbors: &mut Vec<usize>,
    ) {
        let group = self.agent_group(index);
        grid.neighbors(index, observed, group.config().neighbor_radius, neighbors);
        neighbors
            .retain(|&j| group.interaction(observed[j].group_id) != super::Interaction::Ignore);
    }

    fn reconcile_controlled_requests(
        &self,
        observed: &[super::Agent],
        next: &mut [super::Agent],
        grid: &SpatialIndex,
    ) -> Result<(), SimulationError> {
        let dt = self.config.dt;
        let mut neighbors = Vec::new();
        for _ in 0..3 {
            let mut changed = false;
            for i in 0..next.len() {
                let config = self.agent_config(i);
                let command = self.agent_group(i).command();
                if next[i].state != AgentState::Moving
                    || config.separation_weight == 0.0
                    || !matches!(
                        command,
                        Command::Gather { .. } | Command::Spread { .. } | Command::FollowRoute
                    )
                {
                    continue;
                }
                self.visible_neighbors(i, observed, grid, &mut neighbors);
                let safe =
                    |velocity: [f64; 2], all: &[super::Agent]| -> Result<bool, SimulationError> {
                        let ground = add(velocity, next[i].drift);
                        for &j in &neighbors {
                            if all[j].state == AgentState::Terminated {
                                continue;
                            }
                            let relative = sub(observed[i].position, observed[j].position);
                            let velocity = sub(ground, all[j].velocity());
                            if dot(relative, velocity) < 0.0
                                && super::collision::contact_time(
                                    relative,
                                    velocity,
                                    config.body_radius + self.agent_config(j).body_radius,
                                    dt,
                                )?
                                .is_some()
                            {
                                return Ok(false);
                            }
                        }
                        for wall in &self.obstacles {
                            if config.obstacle_weight > 0.0
                                && wall.distance(observed[i].position) <= config.neighbor_radius
                                && wall
                                    .contact(observed[i].position, ground, config.body_radius, dt)?
                                    .is_some()
                            {
                                return Ok(false);
                            }
                        }
                        Ok(true)
                    };
                if safe(next[i].velocity, next)? {
                    continue;
                }
                let original = next[i].velocity;
                let mut replacement = None;
                for sample in 1..=26 {
                    let step = (sample - 1) % 13 + 1;
                    let angle = ((step + 1) / 2) as f64 * std::f64::consts::TAU / 12.0
                        * if step % 2 == 0 { -1.0 } else { 1.0 };
                    let candidate = scale(
                        rotate(original, angle),
                        if sample <= 13 { 1.0 } else { 0.5 },
                    );
                    if safe(candidate, next)? {
                        replacement = Some(candidate);
                        break;
                    }
                }
                let velocity = replacement.unwrap_or([0.0; 2]);
                if velocity != original {
                    next[i].velocity = velocity;
                    next[i].settled_time = 0.0;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
        Ok(())
    }
}
