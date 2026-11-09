use super::{SimulationError, collision, math::*};
use std::f64::consts::TAU;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rectangle {
    center: [f64; 2],
    width: f64,
    length: f64,
    angle: f64,
}

impl Rectangle {
    /// Length runs along angle 0 (positive X); width runs across it.
    pub fn new(center: [f64; 2], width: f64, length: f64) -> Result<Self, SimulationError> {
        if !finite(center)
            || [width, length]
                .iter()
                .any(|v| !v.is_finite() || *v <= 0.0 || !v.powi(2).is_finite())
        {
            return Err(SimulationError::InvalidInput(
                "rectangle needs finite coordinates and positive dimensions",
            ));
        }
        let area = Self {
            center,
            width,
            length,
            angle: 0.0,
        };
        let (lo, hi) = area.bounds(0.0)?;
        if (0..2).any(|axis| lo[axis] >= hi[axis]) {
            return Err(SimulationError::InvalidInput(
                "rectangle is too small at these coordinates",
            ));
        }
        Ok(area)
    }

    pub fn rotated(mut self, angle_radians: f64) -> Result<Self, SimulationError> {
        if !angle_radians.is_finite() {
            return Err(SimulationError::InvalidInput("angle must be finite"));
        }
        self.angle = angle_radians.rem_euclid(TAU);
        if self.vertices().iter().any(|p| !finite(*p)) {
            return Err(SimulationError::NumericOverflow);
        }
        Ok(self)
    }
    pub fn width(self) -> f64 {
        self.width
    }
    pub fn length(self) -> f64 {
        self.length
    }
    pub fn angle(self) -> f64 {
        self.angle
    }
    pub fn vertices(self) -> [[f64; 2]; 4] {
        let x = self.length * 0.5;
        let y = self.width * 0.5;
        [[-x, -y], [x, -y], [x, y], [-x, y]].map(|p| self.to_world(p))
    }
    fn local(self) -> Self {
        Self {
            center: [0.0; 2],
            angle: 0.0,
            ..self
        }
    }
    fn to_local(self, point: [f64; 2]) -> [f64; 2] {
        rotate(sub(point, self.center), -self.angle)
    }
    fn to_world(self, point: [f64; 2]) -> [f64; 2] {
        add(self.center, rotate(point, self.angle))
    }

    pub fn center(self) -> [f64; 2] {
        self.center
    }
    fn bounds(self, margin: f64) -> Result<([f64; 2], [f64; 2]), SimulationError> {
        let half = [self.length * 0.5 + margin, self.width * 0.5 + margin];
        let lo = sub(self.center, half);
        let hi = add(self.center, half);
        if !finite(lo) || !finite(hi) {
            return Err(SimulationError::NumericOverflow);
        }
        Ok((lo, hi))
    }

    pub fn contains(self, point: [f64; 2]) -> bool {
        if self.angle != 0.0 {
            return self.local().contains(self.to_local(point));
        }
        let offset = sub(point, self.center);
        offset[0].abs() <= self.length / 2.0 && offset[1].abs() <= self.width / 2.0
    }

    pub(super) fn distance(self, point: [f64; 2]) -> f64 {
        if self.angle != 0.0 {
            return self.local().distance(self.to_local(point));
        }
        let offset = sub(point, self.center);
        norm([
            (offset[0].abs() - self.length / 2.0).max(0.0),
            (offset[1].abs() - self.width / 2.0).max(0.0),
        ])
    }

    pub(super) fn outside(self, point: [f64; 2], margin: f64) -> Result<[f64; 2], SimulationError> {
        if self.angle != 0.0 {
            let p = self.to_local(point);
            let result = self.local().outside(p, margin)?;
            return Ok(if result == p {
                point
            } else {
                self.to_world(result)
            });
        }
        let (lo, hi) = self.bounds(margin)?;
        if (0..2).any(|axis| point[axis] <= lo[axis] || point[axis] >= hi[axis]) {
            return Ok(point);
        }
        let faces = [
            [lo[0], point[1]],
            [hi[0], point[1]],
            [point[0], lo[1]],
            [point[0], hi[1]],
        ];
        Ok(*faces
            .iter()
            .min_by(|a, b| norm(sub(**a, point)).total_cmp(&norm(sub(**b, point))))
            .unwrap())
    }

    pub(super) fn crosses(
        self,
        start: [f64; 2],
        end: [f64; 2],
        margin: f64,
    ) -> Result<bool, SimulationError> {
        if self.angle != 0.0 {
            return self
                .local()
                .crosses(self.to_local(start), self.to_local(end), margin);
        }
        let (lo, hi) = self.bounds(margin)?;
        let delta = sub(end, start);
        let (mut enter, mut exit) = (0.0_f64, 1.0_f64);
        for axis in 0..2 {
            if delta[axis] == 0.0 {
                if start[axis] <= lo[axis] || start[axis] >= hi[axis] {
                    return Ok(false);
                }
            } else {
                let a = (lo[axis] - start[axis]) / delta[axis];
                let b = (hi[axis] - start[axis]) / delta[axis];
                enter = enter.max(a.min(b));
                exit = exit.min(a.max(b));
            }
        }
        Ok(enter < exit - 1e-12)
    }

    pub(super) fn detour(
        self,
        start: [f64; 2],
        goal: [f64; 2],
        body: f64,
        clearance: f64,
    ) -> Result<[f64; 2], SimulationError> {
        if self.angle != 0.0 {
            let p = self.to_local(start);
            let result = self
                .local()
                .detour(p, self.to_local(goal), body, clearance)?;
            return Ok(if result == p {
                start
            } else {
                self.to_world(result)
            });
        }
        let extra = clearance + 0.1 * body;
        let margin = body + extra;
        let escaped = self.outside(start, margin)?;
        if escaped != start {
            let waypoint = self.detour(escaped, goal, body, clearance)?;
            // Keep tangential progress inside the planning gap, unless that shortcut hits the body.
            return Ok(
                if self
                    .contact(start, sub(waypoint, start), body, 1.0)?
                    .is_none()
                {
                    waypoint
                } else {
                    escaped
                },
            );
        }
        let goal = self.outside(goal, margin)?;
        if !self.crosses(start, goal, margin)? {
            return Ok(goal);
        }
        let (lo, hi) = self.bounds(margin)?;
        let nodes = [start, goal, lo, [lo[0], hi[1]], hi, [hi[0], lo[1]]];
        let mut distances = [f64::INFINITY; 6];
        let mut previous = [0; 6];
        let mut visited = [false; 6];
        distances[0] = 0.0;
        for _ in 0..6 {
            let current = (0..6)
                .filter(|&i| !visited[i])
                .min_by(|&a, &b| distances[a].total_cmp(&distances[b]))
                .unwrap();
            if !distances[current].is_finite() {
                break;
            }
            visited[current] = true;
            for next in 0..6 {
                if visited[next] || self.crosses(nodes[current], nodes[next], margin)? {
                    continue;
                }
                let distance = distances[current] + norm(sub(nodes[next], nodes[current]));
                if distance < distances[next] {
                    distances[next] = distance;
                    previous[next] = current;
                }
            }
        }
        if !distances[1].is_finite() {
            return Ok(start);
        }
        let mut path = vec![1];
        while previous[*path.last().unwrap()] != 0 {
            path.push(previous[*path.last().unwrap()]);
        }
        while path.len() > 1 && norm(sub(nodes[*path.last().unwrap()], start)) <= extra * 0.2 {
            path.pop();
        }
        Ok(nodes[*path.last().unwrap()])
    }

    pub(super) fn contact(
        self,
        position: [f64; 2],
        velocity: [f64; 2],
        radius: f64,
        horizon: f64,
    ) -> Result<Option<f64>, SimulationError> {
        if self.angle != 0.0 {
            return self.local().contact(
                self.to_local(position),
                rotate(velocity, -self.angle),
                radius,
                horizon,
            );
        }
        if self.distance(position) <= radius {
            return Ok(Some(0.0));
        }
        let (lo, hi) = self.bounds(0.0)?;
        let mut earliest: Option<f64> = None;
        let mut record = |time: f64| {
            if time >= 0.0 && time <= horizon {
                earliest = Some(earliest.map_or(time, |old| old.min(time)));
            }
        };
        for axis in 0..2 {
            for (face, sign) in [(lo[axis], -1.0), (hi[axis], 1.0)] {
                if velocity[axis] * sign >= 0.0 {
                    continue;
                }
                let time = (face + sign * radius - position[axis]) / velocity[axis];
                let other = position[1 - axis] + velocity[1 - axis] * time;
                if other >= lo[1 - axis] && other <= hi[1 - axis] {
                    record(time);
                }
            }
        }
        for corner in [lo, [lo[0], hi[1]], hi, [hi[0], lo[1]]] {
            if let Some(time) =
                collision::contact_time(sub(position, corner), velocity, radius, horizon)?
            {
                record(time);
            }
        }
        Ok(earliest)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Square(Rectangle);
impl Square {
    pub fn new(center: [f64; 2], side: f64) -> Result<Self, SimulationError> {
        Ok(Self(Rectangle::new(center, side, side)?))
    }
    pub fn rotated(self, angle: f64) -> Result<Self, SimulationError> {
        Ok(Self(self.0.rotated(angle)?))
    }
    pub fn center(self) -> [f64; 2] {
        self.0.center()
    }
    pub fn side(self) -> f64 {
        self.0.width()
    }
    pub fn angle(self) -> f64 {
        self.0.angle()
    }
    pub fn contains(self, p: [f64; 2]) -> bool {
        self.0.contains(p)
    }
    pub fn vertices(self) -> [[f64; 2]; 4] {
        self.0.vertices()
    }
}
impl From<Square> for Rectangle {
    fn from(square: Square) -> Self {
        square.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn entering_the_planning_gap_preserves_tangential_guidance() {
        let square: Rectangle = Square::new([0.0, 0.0], 2.0).unwrap().into();
        let outside = square.detour([1.18, 0.0], [0.0, 4.0], 0.025, 0.15).unwrap();
        let inside = square.detour([1.11, 0.0], [0.0, 4.0], 0.025, 0.15).unwrap();
        assert!(inside[1] > 1.0 && inside[0] > 1.1, "{inside:?}");
        assert!(
            norm(sub(inside, outside)) < 0.01,
            "inside={inside:?}, outside={outside:?}"
        );
    }

    #[test]
    fn a_corner_recovery_never_guides_through_the_physical_body() {
        for angle in [0.0, 0.49] {
            let rectangle = Rectangle::new([0.0, 0.0], 2.0, 2.0)
                .unwrap()
                .rotated(angle)
                .unwrap();
            let start = rectangle.to_world([1.019, -1.019]);
            let goal = rectangle.to_world([0.0, 4.0]);
            let waypoint = rectangle.detour(start, goal, 0.025, 0.15).unwrap();
            assert_ne!(waypoint, start);
            assert!(
                rectangle
                    .contact(start, sub(waypoint, start), 0.025, 1.0)
                    .unwrap()
                    .is_none(),
                "{waypoint:?}"
            );
        }
    }
}
