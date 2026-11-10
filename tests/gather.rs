use rust_playground::simulation::{Agent, AgentState, Command, Config, Simulation};

fn parked(config: Config, positions: &[[f64; 2]]) -> Simulation {
    let agents = positions
        .iter()
        .map(|p| Agent::new(*p, [1.0, 0.0]).unwrap())
        .collect();
    let mut world = Simulation::new(config, agents).unwrap();
    world.set_command(Command::Hold).unwrap();
    world
}

#[test]
fn fly_target_is_the_center_of_the_area_from_every_direction() {
    for start in [[-4.0, 0.0], [4.0, 0.0], [0.0, -4.0], [0.0, 4.0]] {
        let mut world = parked(Config::default(), &[start]);
        world
            .set_command(Command::Gather {
                center: [0.0, 0.0],
                radius: 1.0,
            })
            .unwrap();
        for _ in 0..200 {
            world.step().unwrap();
        }
        let agent = world.agents()[0];
        assert_eq!(agent.state(), AgentState::Arrived);
        assert!(
            agent.position()[0].hypot(agent.position()[1]) < 0.05,
            "start={start:?}, position={:?}",
            agent.position()
        );
    }
}

#[test]
fn a_small_region_is_accepted_without_fitting_a_preassigned_formation() {
    let mut world = parked(
        Config {
            body_radius: 0.025,
            ..Config::default()
        },
        &[[-4.0, 0.0], [-3.9, 0.0], [4.0, 0.0]],
    );
    world
        .set_command(Command::Gather {
            center: [0.0, 0.0],
            radius: 0.5,
        })
        .unwrap();
    assert_eq!(
        world.command(),
        Command::Gather {
            center: [0.0, 0.0],
            radius: 0.5
        }
    );
}

#[test]
fn a_gather_region_is_a_soft_goal_and_copters_can_park_outside_it() {
    let mut world = parked(Config::default(), &[[-1.0, 0.0], [0.0, 0.0], [1.0, 0.0]]);
    world
        .set_command(Command::Gather {
            center: [0.0, 0.0],
            radius: 0.05,
        })
        .unwrap();
    for _ in 0..400 {
        world.step().unwrap();
    }
    assert!(
        world
            .agents()
            .iter()
            .all(|a| a.state() == AgentState::Arrived),
        "{:?}",
        world.agents()
    );
    assert!(
        world
            .agents()
            .iter()
            .any(|a| a.position()[0].hypot(a.position()[1]) > 0.05)
    );
    for (i, a) in world.agents().iter().enumerate() {
        for b in &world.agents()[..i] {
            assert!(
                (a.position()[0] - b.position()[0]).hypot(a.position()[1] - b.position()[1]) > 0.05
            );
        }
    }
}

#[test]
fn nearby_gather_centers_do_not_cause_collisions_in_the_default_swarm() {
    for seed in [0, 42, 99] {
        for x in [0.0, 0.0875, 0.1] {
            let mut world = Simulation::seeded(Config::default(), 10, 3.0, seed).unwrap();
            world.set_command(Command::Hold).unwrap();
            world
                .set_command(Command::Gather {
                    center: [x, 0.0],
                    radius: 1.0,
                })
                .unwrap();
            for _ in 0..600 {
                world.step().unwrap();
            }
            assert!(
                world
                    .agents()
                    .iter()
                    .all(|a| a.state() == AgentState::Arrived),
                "seed={seed}, center=[{x}, 0]: {:?}",
                world.agents()
            );
        }
    }
}

#[test]
fn a_stationary_center_agent_leaves_free_space_for_another_arrival() {
    let mut world = parked(Config::default(), &[[0.0, 0.0], [-2.0, 0.0]]);
    world
        .set_command(Command::Gather {
            center: [0.0, 0.0],
            radius: 1.0,
        })
        .unwrap();
    for _ in 0..400 {
        world.step().unwrap();
    }
    assert!(
        world
            .agents()
            .iter()
            .all(|a| a.state() == AgentState::Arrived),
        "{:?}",
        world.agents()
    );
    let a = world.agents()[0].position();
    let b = world.agents()[1].position();
    assert!(a[0].hypot(a[1]) < 0.05);
    assert!(b[0].hypot(b[1]) <= 0.975);
    assert!((a[0] - b[0]).hypot(a[1] - b[1]) > 0.05);
}
