use super::{
    Agent, Command, Config, Simulation, SimulationError, actuator::actuate,
    collision::contact_time, math::*,
};
use std::f64::consts::TAU;

impl Simulation {
    pub(super) fn copter_motion(
        &self,
        index: usize,
        goal: [f64; 2],
        visible: &[usize],
        observed: &[Agent],
    ) -> Result<([f64; 2], [f64; 2]), SimulationError> {
        let agent = observed[index];
        let config = self.agent_config(index);
        let command = self.agent_group(index).command();
        let obstacle = self.obstacle.filter(|o| {
            config.obstacle_weight > 0.0 && o.distance(agent.position) <= config.neighbor_radius
        });
        let goal = if let Some(o) = obstacle {
            o.detour(
                agent.position,
                goal,
                config.body_radius,
                config.obstacle_clearance,
            )?
        } else {
            goal
        };
        let offset = sub(goal, agent.position);
        let distance = norm(offset);
        // Leave room for both braking and ramping acceleration down under the jerk bound.
        let braking = config
            .max_braking
            .min((config.max_jerk * distance.max(0.0)).sqrt());
        let mut preferred = [0.0; 2];
        for axis in 0..2 {
            let d = offset[axis].abs();
            preferred[axis] = offset[axis].signum()
                * config
                    .speed
                    .min((2.0 * braking * d).sqrt())
                    .min(d / config.dt)
                * config.command_weight.min(1.0);
        }
        let mut preferred_ground = limited(preferred, config.speed);
        if command == Command::FollowRoute && agent.state == super::AgentState::Moving {
            preferred_ground = scale(
                preferred_ground,
                self.route_speed_factor(index, visible, observed),
            );
        }
        let preferred = if agent.drift == [0.0; 2] {
            preferred_ground
        } else {
            limited(sub(preferred_ground, agent.drift), config.speed)
        };
        let forward = unit(preferred)
            .or_else(|| unit(agent.velocity()))
            .unwrap_or([1.0, 0.0]);
        let neighbors = visible;
        let mut best = None;
        let mut best_cost = f64::INFINITY;
        let candidates =
            if (neighbors.is_empty() || config.separation_weight == 0.0) && obstacle.is_none() {
                1
            } else {
                25
            };
        for sample in 0..candidates {
            let desired = if sample == 24 {
                limited(scale(agent.drift, -1.0), config.speed)
            } else {
                let angle = (sample % 12) as f64 * TAU / 12.0;
                let speed = if sample < 12 {
                    norm(preferred)
                } else {
                    norm(preferred) * 0.5
                };
                [
                    speed * (forward[0] * angle.cos() - forward[1] * angle.sin()),
                    speed * (forward[0] * angle.sin() + forward[1] * angle.cos()),
                ]
            };
            let (velocity, acceleration) = actuate(agent, desired, config)?;
            let mut cost = 0.1 * norm(sub(desired, preferred)).powi(2)
                + norm(sub(velocity, preferred)).powi(2)
                + 0.001 * norm(sub(acceleration, agent.acceleration)).powi(2);
            // Avoidance only adds cost, so this candidate cannot beat the current winner.
            if cost >= best_cost {
                continue;
            }
            if (config.separation_weight > 0.0 && !neighbors.is_empty()) || obstacle.is_some() {
                let horizon = 1.5;
                let prediction_step = config.dt.max(horizon / 29.0);
                let prediction_config = Config {
                    dt: prediction_step,
                    ..config
                };
                let mut predicted = agent;
                let mut elapsed = 0.0;
                let mut first_contact: Option<f64> = None;
                let mut wall_contact: Option<f64> = None;
                // Roll out the same jerk-limited actuator, rather than assuming instant braking.
                for prediction in 0..30 {
                    if elapsed >= horizon {
                        break;
                    }
                    let (v, a) = if prediction == 0 {
                        (velocity, acceleration)
                    } else {
                        actuate(predicted, desired, prediction_config)?
                    };
                    let duration = (if prediction == 0 {
                        config.dt
                    } else {
                        prediction_step
                    })
                    .min(horizon - elapsed);
                    let ground = add(v, predicted.drift);
                    for &j in neighbors.iter().filter(|_| config.separation_weight > 0.0) {
                        let other = &observed[j];
                        let bodies = config.body_radius + self.agent_config(j).body_radius;
                        let safety = (1.4 * bodies).max(config.separation_radius * 0.75);
                        let relative_position = sub(
                            predicted.position,
                            add(other.position, scale(other.velocity(), elapsed)),
                        );
                        let relative_velocity = sub(ground, other.velocity());
                        let approach = dot(relative_position, relative_velocity);
                        if approach >= 0.0 {
                            continue;
                        }
                        if dot(relative_position, relative_position) < safety * safety {
                            // Contact with the buffer is already at t=0; rank recovery by actual approach.
                            let closest_time = (-approach
                                / dot(relative_velocity, relative_velocity))
                            .clamp(0.0, horizon - elapsed);
                            let closest = norm(add(
                                relative_position,
                                scale(relative_velocity, closest_time),
                            ));
                            cost += config.separation_weight.min(1.0)
                                * (1000.0 * (1.0 - closest / safety).max(0.0)
                                    + if closest <= bodies { 1_000_000.0 } else { 0.0 });
                        }
                        if let Some(time) =
                            contact_time(relative_position, relative_velocity, safety, duration)?
                        {
                            let time = elapsed + time;
                            first_contact =
                                Some(first_contact.map_or(time, |previous| previous.min(time)));
                        }
                    }
                    if let Some(o) = obstacle
                        && let Some(time) = o.contact(
                            predicted.position,
                            ground,
                            config.body_radius + config.obstacle_clearance * 0.5,
                            duration,
                        )?
                    {
                        wall_contact = Some(elapsed + time);
                    }
                    if first_contact.is_some() || wall_contact.is_some() {
                        break;
                    }
                    predicted.position = add(predicted.position, scale(ground, duration));
                    predicted.velocity = v;
                    predicted.acceleration = a;
                    elapsed += duration;
                }
                if let Some(time) = first_contact {
                    cost +=
                        config.separation_weight.min(1.0) * (1000.0 + 1000.0 * (horizon - time));
                }
                if let Some(time) = wall_contact {
                    cost += config.obstacle_weight * (1000.0 + 1000.0 * (horizon - time));
                }
            }
            if cost < best_cost {
                best = Some((velocity, acceleration));
                best_cost = cost;
            }
        }
        best.ok_or(SimulationError::NumericOverflow)
    }
}
