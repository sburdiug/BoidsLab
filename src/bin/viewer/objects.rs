use super::{SELECTED, Tool, camera::screen_point};
use macroquad::prelude::*;
use rust_playground::simulation::{NoiseZone, Rectangle, Simulation, WindZone};

pub(super) const OBJECT_LABELS: [&str; 11] = [
    "Width X",
    "Height Y",
    "Angle deg",
    "Wind width",
    "Wind length",
    "Angle deg",
    "Wind speed",
    "Noise width",
    "Noise length",
    "Angle deg",
    "Noise sigma",
];
pub(super) struct ObjectProperties {
    pub values: [String; 11],
}
impl Default for ObjectProperties {
    fn default() -> Self {
        Self {
            values: ["2", "2", "0", "2", "4", "0", "0.3", "2", "2", "0", "0.05"].map(str::to_owned),
        }
    }
}
impl ObjectProperties {
    pub fn fields(tool: Tool) -> &'static [usize] {
        match tool {
            Tool::Wind => &[3, 4, 5, 6],
            Tool::Noise => &[7, 8, 9, 10],
            Tool::Route => &[],
            _ => &[0, 1, 2],
        }
    }
    fn number(&self, i: usize) -> Result<f64, String> {
        self.values[i]
            .trim()
            .parse()
            .map_err(|_| format!("Invalid {}", OBJECT_LABELS[i]))
    }
    pub fn create(
        &self,
        tool: Tool,
        center: [f64; 2],
        dimensions: Option<[f64; 2]>,
    ) -> Result<SceneObject, String> {
        let build = || -> Result<SceneObject, Box<dyn std::error::Error>> {
            Ok(match tool {
                Tool::Rectangle => {
                    let [height, width] = if let Some(d) = dimensions {
                        d
                    } else {
                        [self.number(1)?, self.number(0)?]
                    };
                    SceneObject::Obstacle(
                        Rectangle::new(center, height, width)?
                            .rotated(self.number(2)?.to_radians())?,
                    )
                }
                Tool::Wind => {
                    let [width, length] = if let Some(d) = dimensions {
                        d
                    } else {
                        [self.number(3)?, self.number(4)?]
                    };
                    SceneObject::Wind(WindZone::rectangular(
                        center,
                        width,
                        length,
                        self.number(5)?.to_radians(),
                        self.number(6)?,
                    )?)
                }
                Tool::Noise => {
                    let [width, length] = if let Some(d) = dimensions {
                        d
                    } else {
                        [self.number(7)?, self.number(8)?]
                    };
                    SceneObject::Noise(NoiseZone::new(
                        Rectangle::new(center, width, length)?
                            .rotated(self.number(9)?.to_radians())?,
                        self.number(10)?,
                    )?)
                }
                _ => return Err("Choose a scene object tool".into()),
            })
        };
        build().map_err(|e| e.to_string())
    }
    pub fn load(&mut self, object: SceneObject) {
        let area = object.area();
        let angle = area.angle().to_degrees();
        let pairs: Vec<(usize, f64)> = match object {
            SceneObject::Obstacle(rect) => vec![(0, rect.length()), (1, rect.width()), (2, angle)],
            SceneObject::Wind(wind) => vec![
                (3, area.width()),
                (4, area.length()),
                (5, angle),
                (6, wind.speed()),
            ],
            SceneObject::Noise(noise) => vec![
                (7, area.width()),
                (8, area.length()),
                (9, angle),
                (10, noise.position_stddev()),
            ],
        };
        for (i, value) in pairs {
            self.values[i] = value.to_string();
        }
    }
    pub fn drawn_dimensions(
        &self,
        tool: Tool,
        start: [f64; 2],
        end: [f64; 2],
    ) -> Result<[f64; 2], String> {
        let angle = self
            .number(match tool {
                Tool::Wind => 5,
                Tool::Noise => 9,
                _ => 2,
            })?
            .to_radians();
        if !angle.is_finite() {
            return Err("Angle must be finite".to_owned());
        }
        let delta = [end[0] - start[0], end[1] - start[1]];
        let length = 2.0 * (delta[0] * angle.cos() + delta[1] * angle.sin()).abs();
        let width = 2.0 * (-delta[0] * angle.sin() + delta[1] * angle.cos()).abs();
        Ok([width, length])
    }
}

#[derive(Clone, Copy)]
pub(super) enum SceneObject {
    Obstacle(Rectangle),
    Wind(WindZone),
    Noise(NoiseZone),
}
impl SceneObject {
    pub fn tool(self) -> Tool {
        match self {
            Self::Obstacle(_) => Tool::Rectangle,
            Self::Wind(_) => Tool::Wind,
            Self::Noise(_) => Tool::Noise,
        }
    }
    pub fn area(self) -> Rectangle {
        match self {
            Self::Obstacle(rect) => rect,
            Self::Wind(zone) => zone.area(),
            Self::Noise(zone) => zone.area(),
        }
    }
    pub fn rotated(self, angle: f64) -> Result<Self, String> {
        match self {
            Self::Obstacle(s) => s.rotated(angle).map(Self::Obstacle),
            Self::Wind(w) => w.rotated(angle).map(Self::Wind),
            Self::Noise(n) => n.rotated(angle).map(Self::Noise),
        }
        .map_err(|e| e.to_string())
    }
    pub fn apply(self, world: &mut Simulation, id: Option<ObjectId>) -> Result<ObjectId, String> {
        match (self, id) {
            (Self::Obstacle(square), None | Some(ObjectId::Obstacle)) => {
                world
                    .set_obstacle(Some(square))
                    .map_err(|e| e.to_string())?;
                Ok(ObjectId::Obstacle)
            }
            (Self::Wind(zone), Some(ObjectId::Wind(i))) => {
                world.replace_wind(i, zone).map_err(|e| e.to_string())?;
                Ok(ObjectId::Wind(i))
            }
            (Self::Noise(zone), Some(ObjectId::Noise(i))) => {
                world.replace_noise(i, zone).map_err(|e| e.to_string())?;
                Ok(ObjectId::Noise(i))
            }
            (Self::Wind(zone), None) => {
                let i = world.wind_zones().len();
                world.add_wind(zone);
                Ok(ObjectId::Wind(i))
            }
            (Self::Noise(zone), None) => {
                let i = world.noise_zones().len();
                world.add_noise(zone);
                Ok(ObjectId::Noise(i))
            }
            _ => Err("Selected object type changed".to_owned()),
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
pub(super) enum ObjectId {
    Obstacle,
    Wind(usize),
    Noise(usize),
}
impl ObjectId {
    pub fn get(self, world: &Simulation) -> Option<SceneObject> {
        match self {
            Self::Obstacle => world.obstacle().map(SceneObject::Obstacle),
            Self::Wind(i) => world.wind_zones().get(i).copied().map(SceneObject::Wind),
            Self::Noise(i) => world.noise_zones().get(i).copied().map(SceneObject::Noise),
        }
    }
}
pub(super) fn pick_object(
    world: &Simulation,
    point: [f64; 2],
    after: Option<ObjectId>,
) -> Option<ObjectId> {
    let ids = std::iter::once(ObjectId::Obstacle)
        .chain((0..world.noise_zones().len()).rev().map(ObjectId::Noise))
        .chain((0..world.wind_zones().len()).rev().map(ObjectId::Wind));
    let hits: Vec<_> = ids
        .filter(|id| {
            id.get(world)
                .is_some_and(|object| object.area().contains(point))
        })
        .collect();
    if hits.is_empty() {
        None
    } else {
        Some(
            hits[after
                .and_then(|id| hits.iter().position(|&i| i == id))
                .map_or(0, |i| (i + 1) % hits.len())],
        )
    }
}

pub(super) struct PendingObject {
    pub object: SceneObject,
    pub id: Option<ObjectId>,
    pub dragging: bool,
}

pub(super) fn rotation_handle(
    area: Rectangle,
    camera: [f64; 2],
    zoom: f64,
    viewport: Rect,
) -> Vec2 {
    let center = area.center();
    let distance = area.length() * 0.5 + 18.0 / zoom;
    screen_point(
        [
            center[0] + distance * area.angle().cos(),
            center[1] + distance * area.angle().sin(),
        ],
        camera,
        zoom,
        viewport,
    )
}
pub(super) fn object_outline(area: Rectangle, camera: [f64; 2], zoom: f64, viewport: Rect) {
    let corners = area
        .vertices()
        .map(|p| screen_point(p, camera, zoom, viewport));
    for i in 0..4 {
        let a = corners[i];
        let b = corners[(i + 1) % 4];
        draw_line(a.x, a.y, b.x, b.y, 3.0, SELECTED);
    }
    let center = screen_point(area.center(), camera, zoom, viewport);
    let handle = rotation_handle(area, camera, zoom, viewport);
    draw_line(center.x, center.y, handle.x, handle.y, 1.0, SELECTED);
    draw_circle(handle.x, handle.y, 6.0, SELECTED);
}
#[cfg(test)]
mod tests {
    use super::super::settings::Settings;
    use super::*;
    #[test]
    fn edits_update_the_selected_field_and_overlapping_objects_can_be_picked() {
        let mut world = Settings::default().build().unwrap();
        let mut properties = ObjectProperties::default();
        let wind = properties
            .create(Tool::Wind, [0.0, 0.0], None)
            .unwrap()
            .apply(&mut world, None)
            .unwrap();
        let noise = properties
            .create(Tool::Noise, [0.0, 0.0], None)
            .unwrap()
            .apply(&mut world, None)
            .unwrap();
        assert!(pick_object(&world, [0.0, 0.0], None) == Some(noise));
        assert!(pick_object(&world, [0.0, 0.0], Some(noise)) == Some(wind));
        properties.values[3] = "3".into();
        properties.values[4] = "7".into();
        properties.values[5] = "90".into();
        properties.values[6] = "0.8".into();
        properties
            .create(Tool::Wind, [0.0, 0.0], None)
            .unwrap()
            .apply(&mut world, Some(wind))
            .unwrap();
        assert_eq!(world.wind_zones().len(), 1);
        assert_eq!(world.wind_zones()[0].area().width(), 3.0);
        assert_eq!(world.wind_zones()[0].area().length(), 7.0);
        assert_eq!(world.wind_zones()[0].speed(), 0.8);
        assert!((world.wind_zones()[0].velocity()[1] - 0.8).abs() < 1e-12);
        properties.load(wind.get(&world).unwrap());
        assert_eq!(properties.values[3], "3");
        properties.values[3] = "0".into();
        assert!(properties.create(Tool::Wind, [0.0, 0.0], None).is_err());
        assert_eq!(world.wind_zones()[0].area().width(), 3.0);
    }
    #[test]
    fn rotation_and_handles_use_the_objects_local_axes() {
        let object = SceneObject::Noise(
            NoiseZone::new(Rectangle::new([0.0, 0.0], 2.0, 6.0).unwrap(), 0.05).unwrap(),
        );
        let rotated = object.rotated(std::f64::consts::FRAC_PI_2).unwrap();
        assert_eq!(rotated.area().width(), 2.0);
        assert_eq!(rotated.area().length(), 6.0);
        assert!(rotated.area().contains([0.0, 2.5]));
        let viewport = Rect::new(300.0, 74.0, 900.0, 704.0);
        let center = screen_point([0.0, 0.0], [0.0, 0.0], 80.0, viewport);
        let handle = rotation_handle(rotated.area(), [0.0, 0.0], 80.0, viewport);
        assert!((handle.x - center.x).abs() < 0.01 && handle.y < center.y);
    }
}
