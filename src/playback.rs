//! Playback advances the shared core without coupling model time to rendering.

use crate::simulation::{AgentState, Simulation, SimulationError};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaybackState {
    Paused,
    Running,
    Stopped,
}

pub struct Playback {
    state: PlaybackState,
    speed: f64,
    accumulator: f64,
    previous_positions: Vec<[f64; 2]>,
}

impl Default for Playback {
    fn default() -> Self {
        Self {
            state: PlaybackState::Paused,
            speed: 1.0,
            accumulator: 0.0,
            previous_positions: Vec::new(),
        }
    }
}

impl Playback {
    pub fn state(&self) -> PlaybackState {
        self.state
    }
    pub fn speed(&self) -> f64 {
        self.speed
    }

    /// Model time waiting to be processed, including the fractional interpolation step.
    pub fn backlog_seconds(&self) -> f64 {
        self.accumulator
    }

    pub fn set_speed(&mut self, speed: f64) -> Result<(), SimulationError> {
        if !speed.is_finite() || speed <= 0.0 {
            return Err(SimulationError::InvalidInput(
                "playback speed must be finite and positive",
            ));
        }
        self.speed = speed;
        Ok(())
    }

    pub fn toggle(&mut self) {
        self.state = match self.state {
            PlaybackState::Paused => PlaybackState::Running,
            PlaybackState::Running => PlaybackState::Paused,
            PlaybackState::Stopped => PlaybackState::Stopped,
        };
        self.accumulator = 0.0;
        self.previous_positions.clear();
    }

    pub fn stop(&mut self) {
        self.state = PlaybackState::Stopped;
        self.accumulator = 0.0;
        self.previous_positions.clear();
    }

    pub fn reset(&mut self) {
        self.state = PlaybackState::Paused;
        self.accumulator = 0.0;
        self.previous_positions.clear();
    }

    /// Interpolate only the display; paused/stepped runs show the exact model state.
    pub fn interpolated_positions(&self, world: &Simulation) -> Vec<[f64; 2]> {
        let alpha = (self.accumulator / world.config().dt).clamp(0.0, 1.0);
        world
            .agents()
            .iter()
            .enumerate()
            .map(|(i, agent)| {
                let current = agent.position();
                if self.state == PlaybackState::Running
                    && agent.state() != AgentState::Terminated
                    && let Some(previous) = self.previous_positions.get(i)
                {
                    [
                        previous[0] * (1.0 - alpha) + current[0] * alpha,
                        previous[1] * (1.0 - alpha) + current[1] * alpha,
                    ]
                } else {
                    current
                }
            })
            .collect()
    }

    pub fn single_step(&mut self, world: &mut Simulation) -> Result<(), SimulationError> {
        if self.state != PlaybackState::Stopped {
            self.reset();
            world.step()?;
        }
        Ok(())
    }

    pub fn frame(&mut self, elapsed: f64, world: &mut Simulation) -> Result<(), SimulationError> {
        if self.state != PlaybackState::Running {
            return Ok(());
        }
        let accumulated = self.accumulator + elapsed * self.speed;
        if !elapsed.is_finite() || elapsed < 0.0 || !accumulated.is_finite() {
            self.reset();
            return Err(SimulationError::InvalidInput("invalid elapsed frame time"));
        }
        self.accumulator = accumulated;
        let dt = world.config().dt;
        let started = Instant::now();
        // One step is indivisible; yield between steps so backlog cannot monopolize the UI.
        for _ in 0..8 {
            if self.accumulator + dt * 1e-9 < dt {
                break;
            }
            self.previous_positions.clear();
            self.previous_positions
                .extend(world.agents().iter().map(|agent| agent.position()));
            if let Err(error) = world.step() {
                self.reset();
                return Err(error);
            }
            self.accumulator = (self.accumulator - dt).max(0.0);
            if started.elapsed() >= Duration::from_millis(8) {
                break;
            }
        }
        Ok(())
    }
}
