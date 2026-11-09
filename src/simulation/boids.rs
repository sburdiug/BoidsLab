use super::{Agent, Command, Simulation, SimulationError, math::*};

impl Simulation {
    pub(super) fn steer(
        &self,
        index: usize,
        neighbors: &[usize],
        observed: &[Agent],
        goal: Option<[f64; 2]>,
    ) -> Result<[f64; 2], SimulationError> {
        let agent = observed[index];
        let config = self.agent_config(index);
        let command = self.agent_group(index).command();
        let goal_offset = goal.map(|p| sub(p, agent.position));
        let desired_speed =
            goal_offset.map_or(config.speed, |v| (norm(v) / config.dt).min(config.speed));
        if goal_offset.is_some() && desired_speed < 1e-10 {
            return Ok(limited(scale(agent.drift, -1.0), config.speed));
        }
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
        if matches!(command, Command::Spread { .. }) {
            alignment = [0.0; 2];
            cohesion = [0.0; 2];
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
            | Command::FollowRoute => goal_offset.and_then(unit).unwrap_or([0.0; 2]),
        };
        request = add(request, scale(command, config.command_weight));
        if !finite(request) {
            return Err(SimulationError::NumericOverflow);
        }
        let current = unit(agent.velocity)
            .or_else(|| unit(request))
            .unwrap_or([1.0, 0.0]);
        let mut desired = unit(request).unwrap_or(current);
        if config.obstacle_weight > 0.0 {
            let mut wall: Option<(f64, [f64; 2])> = None;
            for obstacle in &self.obstacles {
                let distance = obstacle.distance(agent.position);
                if distance > config.neighbor_radius {
                    continue;
                }
                let guide = goal.unwrap_or_else(|| {
                    add(
                        agent.position,
                        scale(
                            desired,
                            config.neighbor_radius + obstacle.width().hypot(obstacle.length()),
                        ),
                    )
                });
                if obstacle.crosses(
                    agent.position,
                    guide,
                    config.body_radius + config.obstacle_clearance,
                )? {
                    let waypoint = obstacle.detour(
                        agent.position,
                        guide,
                        config.body_radius,
                        config.obstacle_clearance,
                    )?;
                    if wall.is_none_or(|(old, _)| distance < old) {
                        wall = Some((distance, waypoint));
                    }
                }
            }
            if let Some((_, waypoint)) = wall {
                let detour = unit(sub(waypoint, agent.position)).unwrap_or(desired);
                let strength = config.obstacle_weight.min(1.0);
                desired = unit(add(scale(desired, 1.0 - strength), scale(detour, strength)))
                    .unwrap_or(detour);
                let ground = scale(desired, desired_speed);
                if goal.is_none() {
                    return Ok(ground);
                }
                return self.safe_velocity(
                    index,
                    neighbors,
                    observed,
                    limited(sub(ground, agent.drift), config.speed),
                );
            }
        }
        if let Some(offset) = goal_offset
            && norm(offset) <= 2.0 * config.speed * config.dt
        {
            let velocity = limited(
                sub(scale(offset, 1.0 / config.dt), agent.drift),
                config.speed,
            );
            return self.safe_velocity(index, neighbors, observed, velocity);
        }
        let turn = if goal.is_some() {
            1.0
        } else {
            config.turn_response
        };
        let blended = add(scale(current, 1.0 - turn), scale(desired, turn));
        // Opposite headings at response 0.5 cancel; choose the requested heading.
        let ground = scale(unit(blended).unwrap_or(desired), desired_speed);
        let velocity = if goal.is_some() {
            limited(sub(ground, agent.drift), config.speed)
        } else {
            ground
        };
        if goal.is_none() {
            Ok(velocity)
        } else {
            self.safe_velocity(index, neighbors, observed, velocity)
        }
    }

    fn safe_velocity(
        &self,
        index: usize,
        neighbors: &[usize],
        observed: &[Agent],
        preferred: [f64; 2],
    ) -> Result<[f64; 2], SimulationError> {
        let agent = observed[index];
        let config = self.agent_config(index);
        // Check immediate contacts against visible neighbors; all contacts remain physical.
        for sample in 0..39 {
            let step = sample % 13;
            let angle = if step == 0 {
                0.0
            } else {
                ((step + 1) / 2) as f64 * std::f64::consts::TAU / 12.0
                    * if step % 2 == 0 { -1.0 } else { 1.0 }
            };
            let scale_factor = match sample / 13 {
                0 => 1.0,
                1 => 0.5,
                _ => 0.2,
            };
            let candidate = scale(rotate(preferred, angle), scale_factor);
            let mut safe = true;
            for &j in neighbors.iter().filter(|_| config.separation_weight > 0.0) {
                let other = observed[j];
                let relative_position = sub(agent.position, other.position);
                let relative_velocity = sub(add(candidate, agent.drift), other.velocity());
                if dot(relative_position, relative_velocity) >= 0.0 {
                    continue;
                }
                if super::collision::contact_time(
                    relative_position,
                    relative_velocity,
                    config.body_radius + self.agent_config(j).body_radius,
                    config.dt,
                )?
                .is_some()
                {
                    safe = false;
                    break;
                }
            }
            if safe {
                for wall in &self.obstacles {
                    if config.obstacle_weight > 0.0
                        && wall.distance(agent.position) <= config.neighbor_radius
                        && wall
                            .contact(
                                agent.position,
                                add(candidate, agent.drift),
                                config.body_radius,
                                config.dt,
                            )?
                            .is_some()
                    {
                        safe = false;
                        break;
                    }
                }
            }
            if safe {
                return Ok(candidate);
            }
        }
        Ok([0.0; 2])
    }
}
