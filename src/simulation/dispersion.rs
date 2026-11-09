use super::{Agent, Simulation, SimulationError, math::*};

impl Simulation {
    pub(super) fn dispersion_goal(
        &self,
        index: usize,
        visible: &[usize],
        observed: &[Agent],
        spacing: f64,
    ) -> Result<[f64; 2], SimulationError> {
        let position = observed[index].position;
        let mut pressure = [0.0; 2];
        for &j in visible {
            if !self.reacts_to(index, j) {
                continue;
            }
            let away = sub(position, observed[j].position);
            let distance = norm(away);
            if distance < spacing {
                // At rest, symmetric neighbors may cancel; the outer agents still expand the group.
                let direction =
                    unit(away).unwrap_or(if index < j { [-1.0, 0.0] } else { [1.0, 0.0] });
                pressure = add(pressure, scale(direction, 0.5 * (spacing - distance)));
            }
        }
        let goal = add(position, limited(pressure, spacing));
        if !finite(goal) {
            return Err(SimulationError::NumericOverflow);
        }
        Ok(goal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::{AgentState, Command, Config};

    #[test]
    fn hovering_spread_agents_replan_when_a_neighbor_enters_the_gap() {
        let mut world = Simulation::new(
            Config::default(),
            vec![
                Agent::new([-0.5, 0.0], [1.0, 0.0]).unwrap(),
                Agent::new([0.5, 0.0], [1.0, 0.0]).unwrap(),
            ],
        )
        .unwrap();
        world.set_command(Command::Hold).unwrap();
        world.set_command(Command::Spread { spacing: 0.8 }).unwrap();
        for _ in 0..10 {
            world.step().unwrap();
        }
        assert!(world.agents.iter().all(|a| a.state == AgentState::Arrived));
        world.agents[0].position = [0.2, 0.0];
        world.step().unwrap();
        assert!(world.agents.iter().all(|a| a.state == AgentState::Moving));
        assert!(world.agents[1].velocity()[0] > 0.0);
    }
}
