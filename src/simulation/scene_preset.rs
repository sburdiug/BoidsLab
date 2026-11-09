use super::{EventKind, NoiseZone, Rectangle, Route, Simulation, WindZone};
use std::{fmt::Write as _, fs::OpenOptions, io::Write as _, path::Path};

/// Reusable environment; group state and parameters remain in the receiving simulation.
#[derive(Debug, Clone, PartialEq)]
pub struct ScenePreset {
    obstacles: Vec<Rectangle>,
    wind: Vec<WindZone>,
    noise: Vec<NoiseZone>,
    paths: Vec<Route>,
}

impl ScenePreset {
    pub fn capture(world: &Simulation) -> Self {
        Self {
            obstacles: world.obstacles.clone(),
            wind: world.wind_zones.clone(),
            noise: world.noise_zones.clone(),
            paths: world.paths.clone(),
        }
    }

    pub fn to_text(&self) -> String {
        fn rect(out: &mut String, shape: Rectangle) {
            let [x, y] = shape.center();
            write!(
                out,
                " {x} {y} {} {} {}",
                shape.width(),
                shape.length(),
                shape.angle()
            )
            .unwrap();
        }
        let mut out = String::from("SWARM_SCENE_V1\n");
        for &shape in &self.obstacles {
            out.push_str("obstacle");
            rect(&mut out, shape);
            out.push('\n');
        }
        for &zone in &self.wind {
            out.push_str("wind");
            rect(&mut out, zone.area());
            let [x, y] = zone.velocity();
            writeln!(out, " {x} {y} {}", zone.speed()).unwrap();
        }
        for &zone in &self.noise {
            out.push_str("noise");
            rect(&mut out, zone.area());
            writeln!(out, " {}", zone.position_stddev()).unwrap();
        }
        for route in &self.paths {
            write!(out, "path {}", route.radius()).unwrap();
            for [x, y] in route.points() {
                write!(out, " {x} {y}").unwrap();
            }
            out.push('\n');
        }
        out
    }

    pub fn parse(text: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        if lines.next() != Some("SWARM_SCENE_V1") {
            return Err("not a supported scene preset (expected SWARM_SCENE_V1)".into());
        }
        let mut preset = Self {
            obstacles: Vec::new(),
            wind: Vec::new(),
            noise: Vec::new(),
            paths: Vec::new(),
        };
        for (index, line) in lines.enumerate() {
            if line.trim().is_empty() {
                continue;
            }
            let result = (|| -> Result<(), String> {
                let mut tokens = line.split_whitespace();
                let kind = tokens.next().ok_or("empty line")?;
                let values = tokens
                    .map(|v| v.parse::<f64>().map_err(|_| "invalid number".to_string()))
                    .collect::<Result<Vec<_>, _>>()?;
                let shape = || -> Result<Rectangle, String> {
                    if values.len() < 5 {
                        return Err("missing rectangle coordinates".into());
                    }
                    Rectangle::new([values[0], values[1]], values[2], values[3])
                        .and_then(|r| r.rotated(values[4]))
                        .map_err(|e| e.to_string())
                };
                match kind {
                    "obstacle" if values.len() == 5 => preset.obstacles.push(shape()?),
                    "wind" if values.len() == 8 => preset.wind.push(
                        WindZone::from_preset(shape()?, [values[5], values[6]], values[7])
                            .map_err(|e| e.to_string())?,
                    ),
                    "noise" if values.len() == 6 => preset
                        .noise
                        .push(NoiseZone::new(shape()?, values[5]).map_err(|e| e.to_string())?),
                    "path" if values.len() >= 3 && values.len() % 2 == 1 => {
                        let points = values[1..].as_chunks::<2>().0.to_vec();
                        preset
                            .paths
                            .push(Route::new(points, values[0]).map_err(|e| e.to_string())?);
                    }
                    _ => return Err("unknown entry or wrong number of values".into()),
                }
                Ok(())
            })();
            result.map_err(|e| format!("line {}: {e}", index + 2))?;
        }
        Ok(preset)
    }

    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        Self::parse(&text)
    }

    /// Create a new file without overwriting an existing preset.
    pub fn save_new(&self, path: impl AsRef<Path>) -> Result<(), String> {
        let path = path.as_ref();
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|e| e.to_string())?;
        if let Err(e) = file
            .write_all(self.to_text().as_bytes())
            .and_then(|()| file.flush())
        {
            drop(file);
            let _ = std::fs::remove_file(path);
            return Err(e.to_string());
        }
        Ok(())
    }

    /// Replace the environment and saved paths atomically, preserving groups and their active commands.
    pub fn apply(&self, world: &mut Simulation) -> Result<(), String> {
        let mut next = world.clone();
        next.obstacles.clear();
        for &shape in &self.obstacles {
            next.add_obstacle(shape).map_err(|e| e.to_string())?;
        }
        next.wind_zones = self.wind.clone();
        next.noise_zones = self.noise.clone();
        next.paths = self.paths.clone();
        next.record_scene();
        next.record(EventKind::PathsReplaced {
            paths: self.paths.clone(),
        });
        *world = next;
        Ok(())
    }
}
