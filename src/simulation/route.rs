use super::{Agent, AgentState, Simulation, SimulationError, math::*};

#[derive(Debug, Clone, PartialEq)]
pub struct Route {
    points: Vec<[f64; 2]>,
    radius: f64,
}
impl Route {
    /// `radius` is the corridor half-width; joins and the final gathering area are round.
    pub fn new(points: Vec<[f64; 2]>, radius: f64) -> Result<Self, SimulationError> {
        if points.is_empty()
            || points.iter().any(|p| !finite(*p))
            || points.windows(2).any(|pair| {
                !finite(sub(pair[1], pair[0])) || !norm(sub(pair[1], pair[0])).is_finite()
            })
            || !radius.is_finite()
            || radius <= 0.0
            || !radius.powi(2).is_finite()
        {
            return Err(SimulationError::InvalidInput(
                "route needs finite points and a positive radius",
            ));
        }
        Ok(Self { points, radius })
    }
    pub fn points(&self) -> &[[f64; 2]] {
        &self.points
    }
    pub fn radius(&self) -> f64 {
        self.radius
    }
    /// Full corridor width, matching the UI and `--route-width`.
    pub fn width(&self) -> f64 {
        2.0 * self.radius
    }

    pub(super) fn lateral_offset(&self, index: usize, position: [f64; 2]) -> f64 {
        let segment = index.max(1).min(self.points.len() - 1);
        if segment == 0 {
            return 0.0;
        }
        let direction =
            unit(sub(self.points[segment], self.points[segment - 1])).unwrap_or([0.0; 2]);
        dot(
            sub(position, self.points[segment]),
            [-direction[1], direction[0]],
        )
        .clamp(-self.radius, self.radius)
    }

    pub(super) fn reached(&self, index: usize, position: [f64; 2]) -> bool {
        let tolerance = self.radius * 1e-8;
        if norm(sub(position, self.points[index])) <= self.radius + tolerance {
            return true;
        }
        if index == 0 {
            return false;
        }
        let direction = unit(sub(self.points[index], self.points[index - 1])).unwrap_or([0.0; 2]);
        let offset = sub(position, self.points[index]);
        dot(offset, direction) >= -tolerance
            && dot(offset, [-direction[1], direction[0]]).abs() <= self.radius + tolerance
    }

    pub(super) fn target(
        &self,
        index: usize,
        position: [f64; 2],
        turn_offset: f64,
        centering: f64,
    ) -> [f64; 2] {
        let end = self.points[index];
        if self.points.len() == 1 {
            return end;
        }
        let start = self.points[index.saturating_sub(1)];
        let segment = if index == 0 {
            sub(self.points[1], end)
        } else {
            sub(end, start)
        };
        let length = norm(segment);
        let direction = unit(segment).unwrap_or([0.0; 2]);
        let normal = [-direction[1], direction[0]];
        let along = dot(sub(position, start), direction);
        // Transport the incoming lateral coordinate through the junction, then let lanes vary freely.
        let lateral = if index > 1 && along < 2.0 * self.radius {
            turn_offset
        } else {
            dot(sub(position, start), normal)
        }
        .clamp(-self.radius, self.radius)
            / (1.0 + centering);
        let ahead = if index == 0 {
            0.0
        } else {
            (along + self.radius).clamp(0.0, length)
        };
        add(add(start, scale(direction, ahead)), scale(normal, lateral))
    }

    pub(super) fn end_target(
        &self,
        index: usize,
        position: [f64; 2],
        turn_offset: f64,
        centering: f64,
    ) -> [f64; 2] {
        let target = self.target(index, position, turn_offset, centering);
        if index == 0 {
            return target;
        }
        let direction = unit(sub(self.points[index], self.points[index - 1])).unwrap_or([0.0; 2]);
        add(
            target,
            scale(direction, dot(sub(self.points[index], target), direction)),
        )
    }
}

impl Simulation {
    pub(super) fn route_goal(
        &self,
        index: usize,
        neighbors: &[usize],
        observed: &[Agent],
    ) -> Result<[f64; 2], SimulationError> {
        let agent = observed[index];
        let config = self.agent_config(index);
        let route = self.agent_group(index).route().unwrap();
        if agent.waypoint_index + 1 == route.points.len()
            && (norm(sub(agent.position, *route.points.last().unwrap())) <= 2.0 * route.radius
                || neighbors.iter().any(|&j| {
                    observed[j].group_id == agent.group_id
                        && observed[j].state == AgentState::Arrived
                }))
        {
            return self.local_goal(index, neighbors, observed);
        }
        let target = route.target(
            agent.waypoint_index,
            agent.position,
            agent.route_offset,
            config.route_centering,
        );
        if let Some(obstacle) = self.obstacle
            && config.obstacle_weight > 0.0
            && obstacle.distance(agent.position) <= config.neighbor_radius
            && obstacle.crosses(
                agent.position,
                target,
                config.body_radius + config.obstacle_clearance,
            )?
        {
            return Ok(route.end_target(
                agent.waypoint_index,
                agent.position,
                agent.route_offset,
                config.route_centering,
            ));
        }
        Ok(target)
    }

    pub(super) fn route_speed_factor(
        &self,
        index: usize,
        visible: &[usize],
        observed: &[Agent],
    ) -> f64 {
        let Some(route) = self.agent_group(index).route() else {
            return 1.0;
        };
        let agent = observed[index];
        let config = self.agent_config(index);
        if config.route_cohesion == 0.0
            || agent.waypoint_index == 0
            || visible.is_empty()
            || norm(sub(agent.position, *route.points.last().unwrap())) <= 2.0 * route.radius
            || self.obstacle.is_some_and(|o| {
                config.obstacle_weight > 0.0 && o.distance(agent.position) <= config.neighbor_radius
            })
        {
            return 1.0;
        }
        let tangent = unit(sub(
            route.points[agent.waypoint_index],
            route.points[agent.waypoint_index - 1],
        ))
        .unwrap_or([0.0; 2]);
        let social_count = visible
            .iter()
            .filter(|&&j| self.reacts_to(index, j))
            .count();
        if social_count == 0 {
            return 1.0;
        }
        let mean_along = visible
            .iter()
            .filter(|&&j| self.reacts_to(index, j))
            .map(|&j| dot(sub(observed[j].position, agent.position), tangent))
            .sum::<f64>()
            / social_count as f64;
        // Leading agents ease off for visible followers; no global positions or hard formation slots.
        1.0 / (1.0 + config.route_cohesion * (-mean_along).max(0.0) / config.separation_radius)
    }
}
