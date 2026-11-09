use super::{Agent, Command, Simulation, SimulationError, math::*};

impl Simulation {
    pub(super) fn steer(
        &self,
        index: usize,
        neighbors: &[usize],
        observed: &[Agent],
    ) -> Result<[f64; 2], SimulationError> {
        let agent = observed[index];
        let config = self.agent_config(index);
        let command = self.agent_group(index).command();
        let mut separation = [0.0; 2];
        let mut alignment = [0.0; 2];
        let mut cohesion = [0.0; 2];
        for &j in neighbors {
            let other = &observed[j];
            let offset = sub(other.position, agent.position);
            let distance = norm(offset);
            if self.reacts_to(index, j) {
                cohesion = add(cohesion, offset);
                alignment = add(alignment, unit(other.velocity).unwrap_or([0.0; 2]));
            }
            if distance > 0.0 && distance < config.separation_radius {
                separation = add(
                    separation,
                    scale(
                        offset,
                        -1.0 / distance.max(2.0 * config.body_radius).powi(2),
                    ),
                );
            }
        }
        let weights = [
            config.separation_weight,
            config.alignment_weight,
            config.cohesion_weight,
        ];
        let mut request = [0.0; 2];
        for (vector, weight) in [separation, alignment, cohesion].into_iter().zip(weights) {
            if !finite(vector) {
                return Err(SimulationError::NumericOverflow);
            }
            request = add(request, scale(unit(vector).unwrap_or([0.0; 2]), weight));
        }
        let command = match command {
            Command::Flock => [0.0; 2],
            Command::Move(direction) => direction,
            Command::Gather { .. }
            | Command::Spread { .. }
            | Command::Hold
            | Command::FollowRoute => [0.0; 2],
        };
        request = add(request, scale(command, config.command_weight));
        if !finite(request) {
            return Err(SimulationError::NumericOverflow);
        }
        let current = unit(agent.velocity).ok_or(SimulationError::NumericOverflow)?;
        let mut desired = unit(request).unwrap_or(current);
        if config.obstacle_weight > 0.0
            && let Some(obstacle) = self.obstacle
            && obstacle.distance(agent.position) <= config.neighbor_radius
        {
            let goal = obstacle.detour(
                agent.position,
                // A short guide can end inside a thick wall and project back onto its near face.
                add(
                    agent.position,
                    scale(
                        desired,
                        config.neighbor_radius + obstacle.width().hypot(obstacle.length()),
                    ),
                ),
                config.body_radius,
                config.obstacle_clearance,
            )?;
            desired = unit(sub(goal, agent.position)).unwrap_or(desired);
        }
        let blended = add(
            scale(current, 1.0 - config.turn_response),
            scale(desired, config.turn_response),
        );
        // Opposite headings at response 0.5 cancel; choose the requested heading.
        Ok(scale(unit(blended).unwrap_or(desired), config.speed))
    }
}
