use super::{SimulationError, math::*};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AgentState {
    Moving,
    Arrived,
    Terminated,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Agent {
    pub(super) group_id: usize,
    pub(super) uid: u64,
    pub(super) position: [f64; 2],
    pub(super) velocity: [f64; 2],
    pub(super) settled_time: f64,
    pub(super) state: AgentState,
    pub(super) drift: [f64; 2],
    pub(super) hold_position: [f64; 2],
    pub(super) local_target: Option<[f64; 2]>,
    pub(super) waypoint_index: usize,
    pub(super) route_offset: f64,
}

impl Agent {
    /// Creates a moving agent; the simulation supplies its configured speed.
    pub fn new(position: [f64; 2], direction: [f64; 2]) -> Result<Self, SimulationError> {
        if !finite(position) {
            return Err(SimulationError::InvalidInput("position must be finite"));
        }
        let velocity = unit(direction).ok_or(SimulationError::InvalidInput(
            "direction must be finite and nonzero",
        ))?;
        Ok(Self {
            group_id: 0,
            uid: 0,
            position,
            velocity,
            settled_time: 0.0,
            state: AgentState::Moving,
            drift: [0.0; 2],
            hold_position: position,
            local_target: None,
            waypoint_index: 0,
            route_offset: 0.0,
        })
    }

    /// Stable within a simulation run; storage indices may change.
    pub fn uid(&self) -> u64 {
        self.uid
    }

    pub fn group_id(&self) -> usize {
        self.group_id
    }

    pub fn position(&self) -> [f64; 2] {
        self.position
    }
    pub fn velocity(&self) -> [f64; 2] {
        if self.drift == [0.0; 2] {
            self.velocity
        } else {
            add(self.velocity, self.drift)
        }
    }
    pub fn air_velocity(&self) -> [f64; 2] {
        self.velocity
    }
    pub fn wind_velocity(&self) -> [f64; 2] {
        self.drift
    }
    pub fn waypoint_index(&self) -> usize {
        self.waypoint_index
    }
    pub fn state(&self) -> AgentState {
        self.state
    }
}
