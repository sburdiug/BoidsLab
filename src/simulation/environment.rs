use super::{
    AgentState, Command, EventKind, GroupId, NoiseZone, Rectangle, Route, Simulation,
    SimulationError, WindZone, math::*,
};
use rand::{RngExt, SeedableRng, rngs::StdRng};
use std::f64::consts::TAU;

impl Simulation {
    pub fn obstacle(&self) -> Option<Rectangle> {
        self.obstacles.first().copied()
    }
    pub fn wind_zones(&self) -> &[WindZone] {
        &self.wind_zones
    }
    pub fn noise_zones(&self) -> &[NoiseZone] {
        &self.noise_zones
    }
    pub fn route(&self) -> Option<&Route> {
        self.groups.first().and_then(|g| g.route())
    }

    pub fn obstacles(&self) -> &[Rectangle] {
        &self.obstacles
    }
    fn validate_obstacle(&self, square: Rectangle) -> Result<(), SimulationError> {
        if self.agents.iter().enumerate().any(|(i, a)| {
            a.state != AgentState::Terminated
                && square.distance(a.position) <= self.agent_config(i).body_radius
        }) {
            return Err(SimulationError::InvalidInput(
                "obstacle touches a living agent",
            ));
        }
        for g in &self.groups {
            if g.route.as_ref().is_some_and(|r| {
                r.points()
                    .iter()
                    .any(|p| square.distance(*p) <= g.config.body_radius)
            }) || matches!(g.command, Command::Gather { center, .. } if square.distance(center) <= g.config.body_radius)
            {
                return Err(SimulationError::InvalidInput(
                    "obstacle covers an active group target",
                ));
            }
        }
        Ok(())
    }
    /// Replace all obstacles; retained for single-obstacle callers.
    pub fn set_obstacle(&mut self, obstacle: Option<Rectangle>) -> Result<(), SimulationError> {
        if let Some(shape) = obstacle {
            self.validate_obstacle(shape)?;
        }
        self.obstacles = obstacle.into_iter().collect();
        self.record_scene();
        Ok(())
    }
    pub fn add_obstacle(&mut self, shape: Rectangle) -> Result<usize, SimulationError> {
        self.validate_obstacle(shape)?;
        let index = self.obstacles.len();
        self.obstacles.push(shape);
        self.record_scene();
        Ok(index)
    }
    pub fn replace_obstacle(
        &mut self,
        index: usize,
        shape: Rectangle,
    ) -> Result<(), SimulationError> {
        if index >= self.obstacles.len() {
            return Err(SimulationError::InvalidInput("unknown obstacle"));
        }
        self.validate_obstacle(shape)?;
        self.obstacles[index] = shape;
        self.record_scene();
        Ok(())
    }

    pub fn add_wind(&mut self, zone: WindZone) {
        self.wind_zones.push(zone);
        self.record_scene();
    }
    pub fn add_noise(&mut self, zone: NoiseZone) {
        self.noise_zones.push(zone);
        self.record_scene();
    }
    pub fn replace_wind(&mut self, index: usize, zone: WindZone) -> Result<(), SimulationError> {
        *self
            .wind_zones
            .get_mut(index)
            .ok_or(SimulationError::InvalidInput("unknown wind field"))? = zone;
        self.record_scene();
        Ok(())
    }
    pub fn replace_noise(&mut self, index: usize, zone: NoiseZone) -> Result<(), SimulationError> {
        *self
            .noise_zones
            .get_mut(index)
            .ok_or(SimulationError::InvalidInput("unknown noise field"))? = zone;
        self.record_scene();
        Ok(())
    }
    pub fn clear_fields(&mut self) {
        self.wind_zones.clear();
        self.noise_zones.clear();
        self.record_scene();
    }

    pub fn set_route(&mut self, route: Route) -> Result<(), SimulationError> {
        if self.groups.is_empty() {
            return Err(SimulationError::InvalidInput(
                "add a group before assigning a route",
            ));
        }
        for g in &self.groups {
            for &point in route.points() {
                self.validate_group_target(g.id, point, g.config.body_radius)?;
            }
        }
        for id in 0..self.groups.len() {
            self.set_group_route(id, route.clone())?;
        }
        Ok(())
    }
    pub fn set_group_route(&mut self, id: GroupId, route: Route) -> Result<(), SimulationError> {
        let group = self
            .group(id)
            .ok_or(SimulationError::InvalidInput("unknown group"))?;
        for &point in route.points() {
            self.validate_group_target(id, point, group.config.body_radius)?;
        }
        self.groups[id].route = Some(route);
        self.set_group_command(id, Command::FollowRoute)
    }
    pub(super) fn validate_group_target(
        &self,
        id: GroupId,
        center: [f64; 2],
        body: f64,
    ) -> Result<(), SimulationError> {
        if self.obstacles.iter().any(|o| o.distance(center) <= body) {
            return Err(SimulationError::InvalidInput("target touches the obstacle"));
        }
        if self.agents[self.groups[id].agents.clone()]
            .iter()
            .filter(|a| a.state != AgentState::Terminated)
            .any(|a| !finite(sub(center, a.position)))
        {
            return Err(SimulationError::InvalidInput(
                "target coordinates exceed the numerical range",
            ));
        }
        Ok(())
    }
    pub fn paths(&self) -> &[Route] {
        &self.paths
    }
    pub fn add_path(&mut self, route: Route) -> usize {
        let id = self.paths.len();
        self.paths.push(route.clone());
        self.record(EventKind::PathAdded { path: id, route });
        id
    }
    pub fn assign_path(&mut self, group: GroupId, path: usize) -> Result<(), SimulationError> {
        let route = self
            .paths
            .get(path)
            .ok_or(SimulationError::InvalidInput("unknown path"))?
            .clone();
        self.set_group_route(group, route)
    }

    pub(super) fn wind_at(&self, position: [f64; 2]) -> Result<[f64; 2], SimulationError> {
        let mut wind = [0.0; 2];
        for zone in &self.wind_zones {
            if zone.area.contains(position) {
                wind = add(wind, zone.velocity);
            }
        }
        if !finite(wind) {
            return Err(SimulationError::NumericOverflow);
        }
        Ok(wind)
    }

    pub fn measured_position(&self, index: usize) -> Result<[f64; 2], SimulationError> {
        let mut position = self
            .agents
            .get(index)
            .ok_or(SimulationError::InvalidInput("unknown agent"))?
            .position;
        let actual = position;
        for (field, zone) in self.noise_zones.iter().enumerate() {
            if !zone.area.contains(actual) || zone.position_stddev == 0.0 {
                continue;
            }
            let group = self.agent_group(index);
            let seed = group.spawn.seed
                ^ self.steps.wrapping_mul(0x9e3779b97f4a7c15)
                ^ ((index - group.agents.start) as u64).wrapping_mul(0xbf58476d1ce4e5b9)
                ^ (field as u64).wrapping_mul(0x94d049bb133111eb);
            let mut rng = StdRng::seed_from_u64(seed);
            let radius = (-2.0 * (1.0 - rng.random::<f64>()).ln()).sqrt() * zone.position_stddev;
            let angle = rng.random::<f64>() * TAU;
            position = add(position, [radius * angle.cos(), radius * angle.sin()]);
        }
        if !finite(position) {
            return Err(SimulationError::NumericOverflow);
        }
        Ok(position)
    }
}
