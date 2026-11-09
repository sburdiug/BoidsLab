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
                    let mut neighbors = grid.neighbors(i, &observed, config.neighbor_radius);
                    neighbors.retain(|&j| {
                        self.agent_group(i).interaction(observed[j].group_id)
                            != super::Interaction::Ignore
                    });
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
                            self.local_goal(i, &neighbors, &observed)?
                        };
                        let (velocity, acceleration) =
                            self.copter_motion(i, goal, &neighbors, &observed)?;
                        agent.velocity = velocity;
                        agent.acceleration = acceleration;
                        let slow = norm(agent.velocity()) <= 0.001 * config.speed
                            && norm(acceleration)
                                <= 0.001 * config.max_acceleration.min(config.max_braking);
                        agent.settled_time = if slow {
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
                        agent.velocity = self.steer(i, &neighbors, &observed)?;
                        agent.acceleration = [0.0; 2];
                    }
                }
            }
            resolve_motion(self, &mut next)?;
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
        for change in changes {
            self.record(change);
        }
        Ok(())
    }
}
