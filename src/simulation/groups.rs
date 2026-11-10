use super::{AgentState, Command, Config, EventKind, Route, Simulation, SimulationError, math::*};
use std::ops::Range;

pub type GroupId = usize;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Interaction {
    React,
    Avoid,
    Ignore,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpawnConfig {
    pub count: usize,
    pub center: [f64; 2],
    pub radius: f64,
    pub seed: u64,
}
#[derive(Debug, Clone)]
pub struct Group {
    pub(super) id: GroupId,
    pub(super) uid: u64,
    pub(super) config: Config,
    pub(super) spawn: SpawnConfig,
    pub(super) command: Command,
    pub(super) route: Option<Route>,
    pub(super) agents: Range<usize>,
    pub(super) interactions: Vec<(GroupId, Interaction)>,
    pub(super) profile_hash: u64,
    pub(super) seeded: bool,
}
impl Group {
    pub(super) fn new(
        id: GroupId,
        uid: u64,
        config: Config,
        spawn: SpawnConfig,
        command: Command,
        agents: Range<usize>,
        seeded: bool,
    ) -> Self {
        Self {
            id,
            uid,
            seeded,
            config,
            spawn,
            command,
            agents,
            route: None,
            interactions: Vec::new(),
            profile_hash: config.profile_hash(),
        }
    }
    pub fn is_seeded(&self) -> bool {
        self.seeded
    }
    /// Stable within a simulation run; storage indices may change.
    pub fn uid(&self) -> u64 {
        self.uid
    }

    pub fn id(&self) -> GroupId {
        self.id
    }
    pub fn config(&self) -> Config {
        self.config
    }
    pub fn spawn(&self) -> SpawnConfig {
        self.spawn
    }
    pub fn command(&self) -> Command {
        self.command
    }
    pub fn route(&self) -> Option<&Route> {
        self.route.as_ref()
    }
    pub fn agent_range(&self) -> Range<usize> {
        self.agents.clone()
    }
    pub fn profile_hash(&self) -> u64 {
        self.profile_hash
    }
    pub fn color(&self) -> [u8; 4] {
        let bytes = self.profile_hash.to_le_bytes();
        [
            64 + (bytes[0] as u16 * 191 / 255) as u8,
            64 + (bytes[1] as u16 * 191 / 255) as u8,
            64 + (bytes[2] as u16 * 191 / 255) as u8,
            255,
        ]
    }
    pub fn interaction(&self, other: GroupId) -> Interaction {
        if other == self.id {
            return Interaction::React;
        }
        self.interactions
            .iter()
            .find(|(id, _)| *id == other)
            .map_or(Interaction::React, |(_, mode)| *mode)
    }
    pub fn interactions(&self) -> &[(GroupId, Interaction)] {
        &self.interactions
    }
}
impl Simulation {
    pub fn empty(config: Config) -> Result<Self, SimulationError> {
        config.validate()?;
        Ok(Self {
            config,
            next_agent_uid: 0,
            next_group_uid: 0,
            agents: Vec::new(),
            steps: 0,
            obstacles: Vec::new(),
            wind_zones: Vec::new(),
            noise_zones: Vec::new(),
            groups: Vec::new(),
            paths: Vec::new(),
            events: Vec::new(),
        })
    }
    pub fn groups(&self) -> &[Group] {
        &self.groups
    }
    pub fn group(&self, id: GroupId) -> Option<&Group> {
        self.groups.get(id)
    }
    /// Remove a group and its agents. Later group and agent indices shift down.
    pub fn remove_group(&mut self, id: GroupId) -> Result<(), SimulationError> {
        let range = self
            .group(id)
            .ok_or(SimulationError::InvalidInput("unknown group"))?
            .agents
            .clone();
        let count = range.len();
        self.record(EventKind::GroupRemoved {
            group: id,
            agents: count,
        });
        self.agents.drain(range);
        self.groups.remove(id);
        for group in &mut self.groups {
            if group.id > id {
                group.id -= 1;
                group.agents.start -= count;
                group.agents.end -= count;
            }
            group.interactions.retain(|(target, _)| *target != id);
            for (target, _) in &mut group.interactions {
                if *target > id {
                    *target -= 1;
                }
            }
        }
        for agent in &mut self.agents {
            if agent.group_id > id {
                agent.group_id -= 1;
            }
        }
        Ok(())
    }
    pub fn agent_config(&self, index: usize) -> Config {
        self.groups[self.agents[index].group_id].config
    }
    pub(super) fn agent_group(&self, index: usize) -> &Group {
        &self.groups[self.agents[index].group_id]
    }
    pub fn interaction(
        &self,
        source: GroupId,
        target: GroupId,
    ) -> Result<Interaction, SimulationError> {
        if self.group(target).is_none() {
            return Err(SimulationError::InvalidInput("unknown target group"));
        }
        Ok(self
            .group(source)
            .ok_or(SimulationError::InvalidInput("unknown source group"))?
            .interaction(target))
    }
    pub(super) fn reacts_to(&self, index: usize, other: usize) -> bool {
        self.agent_group(index)
            .interaction(self.agents[other].group_id)
            == Interaction::React
    }
    pub fn perceives(&self, index: usize, other: usize) -> bool {
        if index == other || index >= self.agents.len() || other >= self.agents.len() {
            return false;
        }
        let a = self.agents[index];
        let b = self.agents[other];
        a.state != AgentState::Terminated
            && b.state != AgentState::Terminated
            && self.agent_group(index).interaction(b.group_id) != Interaction::Ignore
            && norm(sub(a.position, b.position)) <= self.agent_config(index).neighbor_radius
    }
    pub fn set_interaction(
        &mut self,
        source: GroupId,
        target: GroupId,
        mode: Interaction,
    ) -> Result<(), SimulationError> {
        self.interaction(source, target)?;
        if source == target {
            return Err(SimulationError::InvalidInput(
                "own-group interaction is always React",
            ));
        }
        let pairs = &mut self.groups[source].interactions;
        if let Some((_, value)) = pairs.iter_mut().find(|(id, _)| *id == target) {
            *value = mode;
        } else {
            pairs.push((target, mode));
            pairs.sort_by_key(|(id, _)| *id);
        }
        self.record(EventKind::InteractionChanged {
            source,
            target,
            mode,
        });
        Ok(())
    }
    pub fn set_time_step_before_start(&mut self, dt: f64) -> Result<(), SimulationError> {
        if !self.groups.is_empty() || self.steps != 0 {
            return Err(SimulationError::InvalidInput(
                "time step is scene-wide; restart an empty scene to change it",
            ));
        }
        let config = Config { dt, ..self.config };
        config.validate()?;
        self.config = config;
        Ok(())
    }
}
