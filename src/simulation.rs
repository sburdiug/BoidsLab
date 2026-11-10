//! Fixed-step 2D motion shared by headless and graphical applications.

mod agent;
mod boids;
mod collision;
mod commands;
mod config;
mod dispersion;
mod environment;
mod error;
mod events;
mod fields;
mod geometry;
mod groups;
mod math;
mod parking;
mod profiles;
mod route;
mod scene_preset;
mod spatial;
mod spawn;
mod step;

pub use agent::{Agent, AgentState};
pub use commands::Command;
pub use config::Config;
pub use error::SimulationError;
pub use events::{Event, EventKind};
pub use fields::{NoiseZone, WindZone};
pub use geometry::{Rectangle, Square};
pub use groups::{Group, GroupId, Interaction, SpawnConfig};
pub use route::Route;
pub use scene_preset::ScenePreset;

#[derive(Debug, Clone)]
pub struct Simulation {
    config: Config,
    next_agent_uid: u64,
    next_group_uid: u64,
    agents: Vec<Agent>,
    steps: u64,
    obstacles: Vec<Rectangle>,
    wind_zones: Vec<WindZone>,
    noise_zones: Vec<NoiseZone>,
    groups: Vec<Group>,
    paths: Vec<Route>,
    events: Vec<Event>,
}

impl Simulation {
    pub fn agents(&self) -> &[Agent] {
        &self.agents
    }
    pub fn config(&self) -> Config {
        self.groups.first().map_or(self.config, |g| g.config())
    }
    pub fn steps(&self) -> u64 {
        self.steps
    }
    pub fn time(&self) -> f64 {
        self.steps as f64 * self.config.dt
    }

    pub fn command(&self) -> Command {
        self.groups.first().map_or(Command::Hold, |g| g.command())
    }
}
