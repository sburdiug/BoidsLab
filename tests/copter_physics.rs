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
fn acceleration_braking_and_jerk_are_physical_limits() {
    let config = Config {
        max_acceleration: 0.6,
        max_braking: 1.4,
        max_jerk: 3.0,
        ..Config::default()
    };
    let mut world = parked(config, &[[0.0, 0.0]]);
    world
        .set_command(Command::Gather {
            center: [20.0, 0.0],
            radius: 1.0,
        })
        .unwrap();
    let mut peak_acceleration: f64 = 0.0;
    let mut peak_braking: f64 = 0.0;
    for step in 0..150 {
        if step == 40 {
            world
                .set_command(Command::Gather {
                    center: [-20.0, 0.0],
                    radius: 1.0,
                })
                .unwrap();
        }
        let old = world.agents()[0];
        world.step().unwrap();
        let a = world.agents()[0];
        let v = a.velocity();
        let acceleration = a.acceleration();
        let previous = old.acceleration();
        assert!(v[0].hypot(v[1]) <= config.speed + 1e-9);
        assert!(
            (acceleration[0] - previous[0]).hypot(acceleration[1] - previous[1])
                <= config.max_jerk * config.dt + 1e-9
        );
        let limit =
            if old.velocity()[0] * acceleration[0] + old.velocity()[1] * acceleration[1] < 0.0 {
                config.max_braking
            } else {
                config.max_acceleration
            };
        assert!(acceleration[0].hypot(acceleration[1]) <= limit + 1e-9);
        if acceleration[0] > 0.0 {
            peak_acceleration = peak_acceleration.max(acceleration[0]);
        }
        if old.velocity()[0] > 0.0 && acceleration[0] < 0.0 {
            peak_braking = peak_braking.max(-acceleration[0]);
        }
    }
    assert!(peak_acceleration > 0.5);
    assert!(peak_braking > peak_acceleration);
}

#[test]
fn a_small_region_is_accepted_without_fitting_a_preassigned_formation() {
    let mut world = parked(Config::default(), &[[-4.0, 0.0], [-3.9, 0.0], [4.0, 0.0]]);
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

#[test]
fn diagonal_retargets_respect_limits_with_different_actuator_settings() {
    for (accel, brake, jerk) in [(0.6, 1.4, 3.0), (2.0, 0.5, 1.0), (1.0, 2.0, 0.5)] {
        let config = Config {
            max_acceleration: accel,
            max_braking: brake,
            max_jerk: jerk,
            ..Config::default()
        };
        let mut world = parked(config, &[[0.0, 0.0]]);
        for step in 0..300 {
            if step % 60 == 0 {
                let center = [
                    [20.0, 10.0],
                    [-10.0, 20.0],
                    [-20.0, -10.0],
                    [10.0, -20.0],
                    [20.0, 10.0],
                ][step / 60];
                world
                    .set_command(Command::Gather {
                        center,
                        radius: 1.0,
                    })
                    .unwrap();
            }
            let old = world.agents()[0];
            world.step().unwrap();
            let agent = world.agents()[0];
            let a = agent.acceleration();
            let previous = old.acceleration();
            assert!((a[0] - previous[0]).hypot(a[1] - previous[1]) <= jerk * config.dt + 1e-9);
            let limit = if old.velocity()[0] * a[0] + old.velocity()[1] * a[1] < 0.0 {
                brake
            } else {
                accel
            };
            assert!(a[0].hypot(a[1]) <= limit + 1e-9);
            assert!(agent.velocity()[0].hypot(agent.velocity()[1]) <= config.speed + 1e-9);
        }
    }
}

#[test]
fn physical_response_is_independent_of_the_boids_turn_response() {
    let mut a = parked(
        Config {
            turn_response: 0.0,
            ..Config::default()
        },
        &[[0.0, 0.0]],
    );
    let mut b = parked(
        Config {
            turn_response: 1.0,
            ..Config::default()
        },
        &[[0.0, 0.0]],
    );
    for world in [&mut a, &mut b] {
        world
            .set_command(Command::Gather {
                center: [10.0, 3.0],
                radius: 1.0,
            })
            .unwrap();
    }
    for _ in 0..50 {
        a.step().unwrap();
        b.step().unwrap();
    }
    assert_eq!(a.agents(), b.agents());
}

#[test]
fn smaller_time_steps_converge_without_changing_the_actuator_limits() {
    let run = |dt| {
        let mut world = parked(
            Config {
                dt,
                ..Config::default()
            },
            &[[0.0, 0.0]],
        );
        world
            .set_command(Command::Gather {
                center: [20.0, 0.0],
                radius: 1.0,
            })
            .unwrap();
        for _ in 0..(4.0 / dt) as usize {
            world.step().unwrap();
        }
        world.agents()[0].position()[0]
    };
    let (a, b, c) = (run(0.1), run(0.05), run(0.025));
    assert!((a - b).abs() < 0.2);
    assert!((b - c).abs() < (a - b).abs());
}
