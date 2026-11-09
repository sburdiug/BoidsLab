use super::{AgentState, Config, EventKind, GroupId, Simulation, SimulationError, math::*};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    Flock,
    Move([f64; 2]),
    Gather { center: [f64; 2], radius: f64 },
    Spread { spacing: f64 },
    Hold,
    FollowRoute,
}

impl Simulation {
    pub(super) fn check_command(
        &self,
        id: GroupId,
        command: Command,
        config: Config,
    ) -> Result<Command, SimulationError> {
        let command = match command {
            Command::Flock | Command::Hold => command,
            Command::FollowRoute => {
                if self.groups[id].route.is_none() {
                    return Err(SimulationError::InvalidInput(
                        "set a route before following it",
                    ));
                }
                command
            }
            Command::Move(direction) => Command::Move(unit(direction).ok_or(
                SimulationError::InvalidInput("command direction must be finite and nonzero"),
            )?),
            Command::Spread { spacing } => {
                if !spacing.is_finite()
                    || spacing < 2.6 * config.body_radius
                    || spacing > config.neighbor_radius
                {
                    return Err(SimulationError::InvalidInput(
                        "neighbor gap must be between 2.6 * body radius and view radius",
                    ));
                }
                command
            }
            Command::Gather { center, radius } => {
                if !finite(center)
                    || !radius.is_finite()
                    || radius <= 0.0
                    || !radius.powi(2).is_finite()
                {
                    return Err(SimulationError::InvalidInput(
                        "gathering region must be finite with positive radius",
                    ));
                }
                command
            }
        };
        if let Command::Gather { center, .. } = command
            && self.agents[self.groups[id].agents.clone()]
                .iter()
                .filter(|a| a.state != AgentState::Terminated)
                .any(|a| !finite(sub(center, a.position)))
        {
            return Err(SimulationError::InvalidInput(
                "region coordinates exceed the supported numerical range",
            ));
        }
        if let Command::Gather { center, .. } = command {
            self.validate_group_target(id, center, config.body_radius)?;
        }
        Ok(command)
    }
    pub fn set_command(&mut self, command: Command) -> Result<(), SimulationError> {
        if self.groups.is_empty() {
            return if command == Command::Hold {
                Ok(())
            } else {
                Err(SimulationError::InvalidInput(
                    "add a group before issuing a command",
                ))
            };
        }
        for g in &self.groups {
            self.check_command(g.id, command, g.config)?;
        }
        for id in 0..self.groups.len() {
            self.set_group_command(id, command)?;
        }
        Ok(())
    }
    pub fn set_group_command(
        &mut self,
        id: GroupId,
        command: Command,
    ) -> Result<(), SimulationError> {
        let group = self
            .group(id)
            .ok_or(SimulationError::InvalidInput("unknown group"))?;
        let config = group.config;
        let command = self.check_command(id, command, config)?;
        let range = self.groups[id].agents.clone();
        self.groups[id].command = command;
        if command != Command::FollowRoute {
            self.groups[id].route = None;
        }
        for agent in &mut self.agents[range] {
            if agent.state == AgentState::Terminated {
                continue;
            }
            if command == Command::Hold {
                agent.state = AgentState::Arrived;
                agent.velocity = limited(scale(agent.drift, -1.0), config.speed);
                agent.acceleration = [0.0; 2];
                agent.settled_time = 0.0;
                agent.hold_position = agent.position;
            } else if matches!(
                command,
                Command::Gather { .. } | Command::Spread { .. } | Command::FollowRoute
            ) {
                agent.state = AgentState::Moving;
                agent.settled_time = 0.0;
                agent.waypoint_index = 0;
                agent.route_offset = 0.0;
            } else if agent.state == AgentState::Arrived || unit(agent.velocity).is_none() {
                let direction = if let Command::Move(direction) = command {
                    direction
                } else {
                    [1.0, 0.0]
                };
                agent.state = AgentState::Moving;
                agent.velocity = scale(direction, config.speed);
                agent.acceleration = [0.0; 2];
            }
        }
        let route = self.groups[id].route.clone();
        self.record(EventKind::CommandChanged {
            group: id,
            command,
            route,
        });
        Ok(())
    }
}
