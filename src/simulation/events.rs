use super::{
    AgentState, Command, Config, GroupId, Interaction, NoiseZone, Rectangle, Route, Simulation,
    SpawnConfig, WindZone,
};
#[derive(Debug, Clone)]
pub struct Event {
    pub step: u64,
    pub kind: EventKind,
}
#[derive(Debug, Clone)]
pub enum EventKind {
    PathAdded {
        path: usize,
        route: Route,
    },
    GroupAdded {
        seeded: bool,
        group: GroupId,
        config: Config,
        spawn: SpawnConfig,
        command: Command,
    },
    ProfileChanged {
        group: GroupId,
        config: Config,
    },
    CommandChanged {
        group: GroupId,
        command: Command,
        route: Option<Route>,
    },
    InteractionChanged {
        source: GroupId,
        target: GroupId,
        mode: Interaction,
    },
    StateChanged {
        agent: usize,
        from: AgentState,
        to: AgentState,
    },
    SceneChanged {
        obstacles: Vec<Rectangle>,
        wind: Vec<WindZone>,
        noise: Vec<NoiseZone>,
    },
}
impl Simulation {
    pub fn events(&self) -> &[Event] {
        &self.events
    }
    pub(super) fn record(&mut self, kind: EventKind) {
        self.events.push(Event {
            step: self.steps,
            kind,
        });
    }
    pub(super) fn record_scene(&mut self) {
        self.record(EventKind::SceneChanged {
            obstacles: self.obstacles.clone(),
            wind: self.wind_zones.clone(),
            noise: self.noise_zones.clone(),
        });
    }
}
