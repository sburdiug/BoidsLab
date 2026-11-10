use super::{
    AgentState, Command, Config, GroupId, Interaction, NoiseZone, Rectangle, Route, Simulation,
    SpawnConfig, WindZone,
};
#[derive(Debug, Clone)]
pub struct Event {
    pub step: u64,
    pub time: f64,
    pub group_uids: Vec<u64>,
    pub agent_uids: Vec<u64>,
    pub kind: EventKind,
}
#[derive(Debug, Clone)]
pub enum EventKind {
    AgentContact {
        first: usize,
        second: usize,
    },
    ObstacleContact {
        agent: usize,
        obstacle: usize,
        shape: Rectangle,
    },
    PathAdded {
        path: usize,
        route: Route,
    },
    PathsReplaced {
        paths: Vec<Route>,
    },
    GroupAdded {
        seeded: bool,
        group: GroupId,
        config: Config,
        spawn: SpawnConfig,
        command: Command,
    },
    GroupRemoved {
        group: GroupId,
        agents: usize,
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
        self.record_at(kind, self.time());
    }
    pub(super) fn record_at(&mut self, kind: EventKind, time: f64) {
        let (groups, agents) =
            match &kind {
                EventKind::GroupAdded { group, .. } | EventKind::GroupRemoved { group, .. } => {
                    (vec![*group], self.groups[*group].agents.clone().collect())
                }
                EventKind::ProfileChanged { group, .. }
                | EventKind::CommandChanged { group, .. } => (vec![*group], vec![]),
                EventKind::InteractionChanged { source, target, .. } => {
                    (vec![*source, *target], vec![])
                }
                EventKind::StateChanged { agent, .. }
                | EventKind::ObstacleContact { agent, .. } => (vec![], vec![*agent]),
                EventKind::AgentContact { first, second } => (vec![], vec![*first, *second]),
                _ => (vec![], vec![]),
            };
        self.events.push(Event {
            step: self.steps,
            time,
            group_uids: groups.into_iter().map(|i| self.groups[i].uid).collect(),
            agent_uids: agents.into_iter().map(|i| self.agents[i].uid).collect(),
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
