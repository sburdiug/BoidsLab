use super::{Agent, AgentState, math::*};
use std::collections::HashMap;

/// The grid is a broad phase only; callers retain their exact distance checks.
// shortcut: dense buckets still require pairwise work; profile a finer grid before raising UI limits.
pub(super) struct SpatialIndex {
    width: f64,
    cells: Option<HashMap<(i64, i64), Vec<usize>>>,
}

impl SpatialIndex {
    pub(super) fn new(agents: &[Agent], width: f64) -> Self {
        let mut grid = Self {
            width,
            cells: Some(HashMap::new()),
        };
        for (i, agent) in agents.iter().enumerate() {
            if agent.state == AgentState::Terminated {
                continue;
            }
            let Some(key) = grid.key(agent.position) else {
                // Preserve the brute-force numerical checks outside the grid's safe range.
                grid.cells = None;
                break;
            };
            grid.cells.as_mut().unwrap().entry(key).or_default().push(i);
        }
        grid
    }

    fn key(&self, point: [f64; 2]) -> Option<(i64, i64)> {
        let cell = [point[0] / self.width, point[1] / self.width];
        (self.width.is_finite()
            && self.width > 0.0
            && point.iter().all(|v| v.abs() < 1e75)
            && cell.iter().all(|v| v.is_finite() && v.abs() < 1e12))
        .then(|| (cell[0].floor() as i64, cell[1].floor() as i64))
    }

    pub(super) fn candidates(&self, point: [f64; 2], agents: &[Agent]) -> Vec<usize> {
        let mut result = Vec::new();
        if let Some(cells) = &self.cells
            && let Some((x, y)) = self.key(point)
        {
            // One guard cell covers inclusive distances rounded across a cell boundary.
            for dx in -2..=2 {
                for dy in -2..=2 {
                    if let Some(indices) = cells.get(&(x + dx, y + dy)) {
                        result.extend_from_slice(indices);
                    }
                }
            }
            result.sort_unstable();
        } else {
            result.extend(
                agents
                    .iter()
                    .enumerate()
                    .filter(|(_, a)| a.state != AgentState::Terminated)
                    .map(|(i, _)| i),
            );
        }
        result
    }

    pub(super) fn neighbors(&self, index: usize, agents: &[Agent], radius: f64) -> Vec<usize> {
        let point = agents[index].position;
        let mut result = self.candidates(point, agents);
        result.retain(|&i| i != index && norm(sub(point, agents[i].position)) <= radius);
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::{Config, Simulation};

    #[test]
    fn seeded_neighborhoods_match_all_pairs_and_exclude_terminated_agents() {
        for seed in [0, 42, 99] {
            let mut agents = Simulation::seeded(Config::default(), 250, 5.0, seed)
                .unwrap()
                .agents;
            agents[5].state = AgentState::Terminated;
            for radius in [0.1, 1.0, 6.0] {
                let grid = SpatialIndex::new(&agents, radius);
                for (i, agent) in agents.iter().enumerate() {
                    let expected: Vec<_> = agents
                        .iter()
                        .enumerate()
                        .filter(|(j, other)| {
                            *j != i
                                && other.state != AgentState::Terminated
                                && norm(sub(agent.position, other.position)) <= radius
                        })
                        .map(|(j, _)| j)
                        .collect();
                    assert_eq!(grid.neighbors(i, &agents, radius), expected);
                }
            }
        }
    }

    #[test]
    fn grid_matches_brute_force_at_boundaries_and_negative_coordinates() {
        let points = [
            [-1.0, 0.0],
            [0.0, 0.0],
            [1.0, 0.0],
            [0.999999, 0.000001],
            [-0.001, -1.0],
            [-1e-17, 0.0],
            [5.0, 5.0],
            [1e100, 1e100],
        ];
        for points in [&points[..7], &points[..]] {
            let agents: Vec<_> = points
                .iter()
                .map(|p| Agent::new(*p, [1.0, 0.0]).unwrap())
                .collect();
            let grid = SpatialIndex::new(&agents, 1.0);
            for (i, a) in agents.iter().enumerate() {
                let expected: Vec<_> = agents
                    .iter()
                    .enumerate()
                    .filter(|(j, b)| *j != i && norm(sub(a.position, b.position)) <= 1.0)
                    .map(|(j, _)| j)
                    .collect();
                assert_eq!(grid.neighbors(i, &agents, 1.0), expected);
            }
        }
    }
}
