use super::{Rectangle, SimulationError, math::*};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WindZone {
    pub(super) area: Rectangle,
    pub(super) velocity: [f64; 2],
    speed: f64,
}
impl WindZone {
    pub(super) fn from_preset(
        area: Rectangle,
        velocity: [f64; 2],
        speed: f64,
    ) -> Result<Self, SimulationError> {
        let mut zone = Self::new(area, velocity)?;
        if !speed.is_finite() || speed < 0.0 || (zone.speed - speed).abs() > 1e-12 * speed.max(1.0)
        {
            return Err(SimulationError::InvalidInput(
                "wind speed and velocity do not match",
            ));
        }
        zone.speed = speed;
        Ok(zone)
    }
    pub fn rectangular(
        center: [f64; 2],
        width: f64,
        length: f64,
        angle: f64,
        speed: f64,
    ) -> Result<Self, SimulationError> {
        if !speed.is_finite() || speed < 0.0 {
            return Err(SimulationError::InvalidInput(
                "wind speed must be finite and nonnegative",
            ));
        }
        let area = Rectangle::new(center, width, length)?.rotated(angle)?;
        let mut wind = Self::new(area, rotate([speed, 0.0], area.angle()))?;
        wind.speed = speed;
        Ok(wind)
    }
    pub fn rotated(self, angle: f64) -> Result<Self, SimulationError> {
        let area = self.area.rotated(angle)?;
        let direction = self.velocity[1].atan2(self.velocity[0]) + area.angle() - self.area.angle();
        Ok(Self {
            area,
            velocity: rotate([self.speed, 0.0], direction),
            speed: self.speed,
        })
    }

    pub fn new(area: impl Into<Rectangle>, velocity: [f64; 2]) -> Result<Self, SimulationError> {
        if !finite(velocity) || !norm(velocity).is_finite() {
            return Err(SimulationError::InvalidInput(
                "wind velocity must be finite",
            ));
        }
        Ok(Self {
            area: area.into(),
            velocity,
            speed: norm(velocity),
        })
    }
    pub fn area(self) -> Rectangle {
        self.area
    }
    pub fn velocity(self) -> [f64; 2] {
        self.velocity
    }
    pub fn speed(self) -> f64 {
        self.speed
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NoiseZone {
    pub(super) area: Rectangle,
    pub(super) position_stddev: f64,
}
impl NoiseZone {
    pub fn rotated(self, angle: f64) -> Result<Self, SimulationError> {
        Self::new(self.area.rotated(angle)?, self.position_stddev)
    }

    pub fn new(area: impl Into<Rectangle>, position_stddev: f64) -> Result<Self, SimulationError> {
        if !position_stddev.is_finite() || position_stddev < 0.0 {
            return Err(SimulationError::InvalidInput(
                "position noise must be finite and nonnegative",
            ));
        }
        Ok(Self {
            area: area.into(),
            position_stddev,
        })
    }
    pub fn area(self) -> Rectangle {
        self.area
    }
    pub fn position_stddev(self) -> f64 {
        self.position_stddev
    }
}
