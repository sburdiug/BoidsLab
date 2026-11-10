use super::{Agent, AgentState, Command, Simulation, SimulationError, math::*};
use std::f64::consts::{PI, TAU};

impl Simulation {
    pub(super) fn parking_path_blocked(
        &self,
        index: usize,
        point: [f64; 2],
        visible: &[usize],
        observed: &[Agent],
    ) -> bool {
        let agent = observed[index];
        let config = self.agent_config(index);
        let path = sub(point, agent.position);
        let direction = unit(path).unwrap_or([0.0; 2]);
        visible.iter().any(|&j| {
            let other = observed[j];
            if other.state != AgentState::Arrived {
                return false;
            }
            let offset = sub(other.position, agent.position);
            if dot(offset, path) <= 0.0 {
                return false;
            }
            let along = dot(offset, direction).clamp(0.0, norm(path));
            norm(sub(offset, scale(direction, along)))
                < (1.4 * (config.body_radius + self.agent_config(j).body_radius))
                    .max(0.75 * config.separation_radius)
        })
    }

    pub(super) fn local_goal(
        &self,
        index: usize,
        visible: &[usize],
        observed: &[Agent],
    ) -> Result<[f64; 2], SimulationError> {
        if self.agents[index].state != AgentState::Moving {
            return Ok(self.agents[index].position);
        }
        let agent = observed[index];
        let config = self.agent_config(index);
        let command = self.agent_group(index).command();
        let (center, radius) = match command {
            Command::Gather { center, radius } => (center, radius),
            Command::FollowRoute => {
                let route = self.agent_group(index).route().unwrap();
                (route.points()[agent.waypoint_index], route.radius())
            }
            _ => return Ok(self.agents[index].position),
        };
        let radial = sub(agent.position, center);
        let distance = norm(radial);
        let usable = (radius - config.body_radius).max(0.0);
        let base = if distance <= usable {
            agent.position
        } else if command == Command::FollowRoute {
            add(center, scale(unit(radial).unwrap_or([1.0, 0.0]), usable))
        } else {
            center
        };
        let count = observed[self.agent_group(index).agent_range()]
            .iter()
            .filter(|a| a.state != AgentState::Terminated)
            .count()
            .max(1);
        let spacing = config
            .separation_radius
            .max(2.6 * config.body_radius)
            .max((radius * (PI / count as f64).sqrt() * 0.8).min(config.neighbor_radius * 0.9));
        let pair_spacing = |a: &Agent| {
            spacing.max(1.3 * (config.body_radius + self.groups[a.group_id].config.body_radius))
        };
        let neighbors: Vec<_> = visible
            .iter()
            .copied()
            .filter(|&j| {
                let other = &observed[j];
                let other_distance = norm(sub(other.position, center));
                other.state == AgentState::Arrived
                    || other_distance < distance - 1e-9
                    || ((other_distance - distance).abs() <= 1e-9 && j < index)
            })
            .map(|j| &observed[j])
            .collect();
        let free = |point: [f64; 2]| {
            finite(point)
                && self.obstacles.iter().all(|o| {
                    config.obstacle_weight == 0.0
                        || o.distance(agent.position) > config.neighbor_radius
                        || o.distance(point) > config.body_radius + config.obstacle_clearance
                })
                && neighbors
                    .iter()
                    .all(|a| norm(sub(point, a.position)) >= pair_spacing(a) * (1.0 - 1e-8))
        };
        let blocked = |point| self.parking_path_blocked(index, point, visible, observed);
        // Do not cross a parked swarm to reach the preferred point.
        if free(base) && !blocked(base) {
            return Ok(base);
        }
        let lower_cost = |point: [f64; 2]| {
            let radial = norm(sub(point, center));
            let path = sub(point, agent.position);
            let center_cost = if command == Command::FollowRoute {
                (radial - radius).max(0.0)
            } else {
                radial
            };
            center_cost + 0.05 * norm(path)
        };
        let cost = |point| lower_cost(point) + if blocked(point) { 1000.0 } else { 0.0 };
        let mut goal = base;
        let mut best = if free(base) {
            cost(base)
        } else {
            f64::INFINITY
        };
        if lower_cost(agent.position) < best && free(agent.position) && cost(agent.position) < best
        {
            goal = agent.position;
            best = cost(goal);
        }
        for obstacle in &self.obstacles {
            let candidate =
                obstacle.outside(center, config.body_radius * 1.1 + config.obstacle_clearance)?;
            if free(candidate) && cost(candidate) < best {
                goal = candidate;
                best = cost(candidate);
            }
        }
        let offsets: [[f64; 2]; 16] = std::array::from_fn(|sample| {
            let angle = sample as f64 * TAU / 16.0;
            [angle.cos(), angle.sin()]
        });
        for other in &neighbors {
            for offset in offsets {
                let candidate = add(other.position, scale(offset, pair_spacing(other) * 1.001));
                let lower = lower_cost(candidate);
                if lower < best && free(candidate) {
                    let score = lower + if blocked(candidate) { 1000.0 } else { 0.0 };
                    if score < best {
                        goal = candidate;
                        best = score;
                    }
                }
            }
        }
        if blocked(goal)
            && super::collision::near_region(index, observed, center, radius, config)
            && visible.iter().all(|&j| {
                norm(sub(agent.position, observed[j].position))
                    > 1.1 * (config.body_radius + self.agent_config(j).body_radius)
            })
            && self
                .obstacles
                .iter()
                .all(|wall| wall.distance(agent.position) > config.body_radius)
        {
            // A blocked destination is not a reason to keep flying through a parked neighbor.
            goal = agent.position;
        }
        if !finite(goal) {
            return Err(SimulationError::NumericOverflow);
        }
        Ok(goal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::Config;

    #[test]
    fn a_parked_neighbor_blocks_only_paths_toward_it() {
        let mut world = Simulation::new(
            Config::default(),
            vec![
                Agent::new([0.0, 0.0], [1.0, 0.0]).unwrap(),
                Agent::new([0.25, 0.0], [1.0, 0.0]).unwrap(),
            ],
        )
        .unwrap();
        world.set_command(Command::Hold).unwrap();
        let agents = world.agents();
        assert!(world.parking_path_blocked(0, [1.0, 0.0], &[1], agents));
        assert!(!world.parking_path_blocked(0, [-1.0, 0.0], &[1], agents));
        assert!(!world.parking_path_blocked(0, [0.0, 0.0], &[1], agents));
    }
}
