use super::SimulationError;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    pub dt: f64,
    pub speed: f64,

    pub turn_response: f64,
    pub neighbor_radius: f64,
    pub separation_radius: f64,
    pub body_radius: f64,
    pub separation_weight: f64,
    pub alignment_weight: f64,
    pub cohesion_weight: f64,
    pub command_weight: f64,
    pub obstacle_clearance: f64,
    pub obstacle_weight: f64,
    pub route_centering: f64,
    pub route_cohesion: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            dt: 0.1,
            speed: 1.0,

            turn_response: 0.1,
            neighbor_radius: 1.0,
            separation_radius: 0.3,
            body_radius: 0.025,
            separation_weight: 10.0,
            alignment_weight: 1.0,
            cohesion_weight: 3.0,
            command_weight: 15.0,
            obstacle_clearance: 0.15,
            obstacle_weight: 1.0,
            route_centering: 0.25,
            route_cohesion: 1.0,
        }
    }
}

impl Config {
    pub(super) fn validate(self) -> Result<(), SimulationError> {
        let positive = [
            self.dt,
            self.speed,
            self.neighbor_radius,
            self.separation_radius,
            self.body_radius,
        ];
        let weights = [
            self.separation_weight,
            self.alignment_weight,
            self.cohesion_weight,
            self.command_weight,
            self.obstacle_clearance,
            self.obstacle_weight,
            self.route_centering,
            self.route_cohesion,
        ];
        if positive.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || weights.iter().any(|v| !v.is_finite() || *v < 0.0)
            || !self.turn_response.is_finite()
            || !(0.0..=1.0).contains(&self.turn_response)
            || self.separation_radius > self.neighbor_radius
            || !(self.speed * self.dt).is_finite()
            || !(2.0 * self.body_radius).powi(2).is_finite()
            || (2.0 * self.body_radius).powi(2) == 0.0
        {
            return Err(SimulationError::InvalidInput(
                "invalid motion parameters, radii, or rule weights",
            ));
        }
        Ok(())
    }
}
