use rust_playground::simulation::{Agent, AgentState, Command, Config, Simulation};

fn agent(position: [f64; 2], direction: [f64; 2]) -> Agent {
    Agent::new(position, direction).unwrap()
}

fn inertial_config() -> Config {
    Config {
        separation_weight: 0.0,
        alignment_weight: 0.0,
        cohesion_weight: 0.0,
        ..Config::default()
    }
}

fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() < 1e-10, "{actual} != {expected}");
}

#[test]
fn one_agent_matches_uniform_motion_and_preserves_speed() {
    let config = Config {
        speed: 2.0,
        ..inertial_config()
    };
    let mut world = Simulation::new(config, vec![agent([-2.0, 3.0], [3.0, 4.0])]).unwrap();
    for _ in 0..10 {
        world.step().unwrap();
        let [x, y] = world.agents()[0].velocity();
        close(x.hypot(y), 2.0);
    }
    close(world.agents()[0].position()[0], -0.8);
    close(world.agents()[0].position()[1], 4.6);
    close(world.time(), 1.0);
    assert_eq!(world.steps(), 10);
}

#[test]
fn copters_accelerate_independently_instead_of_sharing_an_arrival_clock() {
    let mut world = Simulation::new(
        Config::default(),
        vec![
            agent([-3.0, 0.0], [1.0, 0.0]),
            agent([0.0, 2.0], [1.0, 0.0]),
        ],
    )
    .unwrap();
    world.set_command(Command::Hold).unwrap();
    world
        .set_command(Command::Gather {
            center: [3.0, 0.0],
            radius: 1.0,
        })
        .unwrap();
    world.step().unwrap();
    for a in world.agents() {
        close(
            a.velocity()[0].hypot(a.velocity()[1]),
            Config::default().max_jerk * Config::default().dt.powi(2),
        );
    }
    for _ in 0..9 {
        world.step().unwrap();
    }
    for a in world.agents() {
        let v = a.velocity();
        assert!(v[0].hypot(v[1]) > 0.8 && v[0].hypot(v[1]) <= 1.0);
    }
    let mut straight =
        Simulation::new(Config::default(), vec![agent([0.0, 0.0], [1.0, 0.0])]).unwrap();
    straight.set_command(Command::Hold).unwrap();
    straight
        .set_command(Command::Gather {
            center: [20.0, 0.0],
            radius: 1.0,
        })
        .unwrap();
    for _ in 0..10 {
        straight.step().unwrap();
    }
    assert!(straight.agents()[0].velocity()[0] > 0.9);
}

#[test]
fn reversing_on_one_axis_brakes_through_zero_without_a_turn_on_the_other_axis() {
    for axis in 0..2 {
        let position = [2.0, 3.0];
        let mut direction = [0.0; 2];
        direction[axis] = 1.0;
        let mut target = position;
        target[axis] -= 10.0;
        let mut world =
            Simulation::new(Config::default(), vec![agent(position, direction)]).unwrap();
        world
            .set_command(Command::Gather {
                center: target,
                radius: 1.0,
            })
            .unwrap();
        let mut slowest = f64::INFINITY;
        let mut previous = 1.0;
        for _ in 0..30 {
            world.step().unwrap();
            let a = world.agents()[0];
            close(a.velocity()[1 - axis], 0.0);
            close(a.position()[1 - axis], position[1 - axis]);
            assert!(
                (a.velocity()[axis] - previous).abs()
                    <= Config::default().max_braking * Config::default().dt + 1e-10
            );
            slowest = slowest.min(a.velocity()[axis].abs());
            previous = a.velocity()[axis];
        }
        assert!(slowest < Config::default().max_braking * Config::default().dt);
        assert!(world.agents()[0].velocity()[axis] < 0.0);
    }
}

#[test]
fn a_reversal_brakes_under_the_jerk_limit_before_backwards_motion() {
    let config = Config {
        command_weight: 0.43,
        ..Config::default()
    };
    let mut world = Simulation::new(config, vec![agent([0.0, 0.0], [1.0, 0.0])]).unwrap();
    world.set_command(Command::Hold).unwrap();
    world
        .set_command(Command::Gather {
            center: [20.0, 0.0],
            radius: 1.0,
        })
        .unwrap();
    for _ in 0..30 {
        world.step().unwrap();
    }
    assert!((world.agents()[0].velocity()[0] - 0.43).abs() < 0.01);
    world
        .set_command(Command::Gather {
            center: [-20.0, 0.0],
            radius: 1.0,
        })
        .unwrap();
    for _ in 0..30 {
        let previous = world.agents()[0];
        world.step().unwrap();
        let a = world.agents()[0];
        close(a.velocity()[1], 0.0);
        assert!(
            (a.acceleration()[0] - previous.acceleration()[0]).abs()
                <= config.max_jerk * config.dt + 1e-9
        );
        assert!(
            (a.velocity()[0] - previous.velocity()[0]).abs()
                <= config.max_braking * config.dt + 1e-9
        );
    }
    assert!(world.agents()[0].velocity()[0] < 0.0);
}

#[test]
fn a_copter_stabilizes_one_axis_while_still_travelling_on_the_other() {
    for axis in 0..2 {
        let mut target = [100.0, 100.0];
        target[axis] = 1.0;
        let mut world =
            Simulation::new(Config::default(), vec![agent([0.0, 0.0], [1.0, 0.0])]).unwrap();
        world.set_command(Command::Hold).unwrap();
        world
            .set_command(Command::Gather {
                center: target,
                radius: 1.0,
            })
            .unwrap();
        for _ in 0..100 {
            world.step().unwrap();
        }
        let a = world.agents()[0];
        assert!(
            (a.position()[axis] - 1.0).abs() < 0.005,
            "axis not stabilized: {:?}",
            a.position()
        );
        assert!(a.velocity()[axis].abs() < 0.01);
        assert!(a.velocity()[1 - axis] > 0.9);
        assert_eq!(a.state(), AgentState::Moving);
    }
}

#[test]
fn insufficient_local_vision_can_cause_a_controlled_flight_collision() {
    let config = Config {
        neighbor_radius: 0.01,
        separation_radius: 0.005,
        ..Config::default()
    };
    let mut world = Simulation::new(
        config,
        vec![
            agent([-0.1, 0.0], [1.0, 0.0]),
            agent([0.1, 0.0], [-1.0, 0.0]),
        ],
    )
    .unwrap();
    world
        .set_command(Command::Gather {
            center: [4.0, 0.0],
            radius: 1.0,
        })
        .unwrap();
    world.step().unwrap();
    assert!(
        world
            .agents()
            .iter()
            .all(|a| a.state() == AgentState::Terminated)
    );
}

#[test]
fn a_direction_command_steers_instead_of_overwriting_velocity() {
    let mut world =
        Simulation::new(inertial_config(), vec![agent([0.0, 0.0], [1.0, 0.0])]).unwrap();
    world.set_command(Command::Move([0.0, 1.0])).unwrap();
    assert_eq!(world.agents()[0].velocity(), [1.0, 0.0]);
    world.step().unwrap();
    let [x, y] = world.agents()[0].velocity();
    assert!(x > 0.0 && y > 0.0);
    close(x.hypot(y), 1.0);
}

#[test]
fn an_exact_reverse_turn_remains_finite() {
    let config = Config {
        turn_response: 0.5,
        ..inertial_config()
    };
    let mut world = Simulation::new(config, vec![agent([0.0, 0.0], [1.0, 0.0])]).unwrap();
    world.set_command(Command::Move([-1.0, 0.0])).unwrap();
    world.step().unwrap();
    close(world.agents()[0].velocity()[0], -1.0);
}

#[test]
fn each_boids_rule_has_its_expected_effect() {
    let base = Config {
        dt: 0.01,
        turn_response: 1.0,
        ..inertial_config()
    };
    let mut separation = Simulation::new(
        Config {
            separation_weight: 1.0,
            ..base
        },
        vec![
            agent([-0.1, 0.0], [0.0, 1.0]),
            agent([0.1, 0.0], [0.0, 1.0]),
        ],
    )
    .unwrap();
    separation.step().unwrap();
    assert!(separation.agents()[0].velocity()[0] < 0.0);
    assert!(separation.agents()[1].velocity()[0] > 0.0);

    let mut alignment = Simulation::new(
        Config {
            alignment_weight: 1.0,
            ..base
        },
        vec![agent([0.0, 0.0], [1.0, 0.0]), agent([0.0, 0.5], [0.0, 1.0])],
    )
    .unwrap();
    alignment.step().unwrap();
    assert_eq!(alignment.agents()[0].velocity(), [0.0, 1.0]);
    assert_eq!(alignment.agents()[1].velocity(), [1.0, 0.0]);

    let mut cohesion = Simulation::new(
        Config {
            cohesion_weight: 1.0,
            ..base
        },
        vec![
            agent([-0.4, 0.0], [0.0, 1.0]),
            agent([0.4, 0.0], [0.0, 1.0]),
        ],
    )
    .unwrap();
    cohesion.step().unwrap();
    assert!(cohesion.agents()[0].velocity()[0] > 0.0);
    assert!(cohesion.agents()[1].velocity()[0] < 0.0);
}

#[test]
fn seeded_runs_repeat_in_the_same_environment() {
    let mut first = Simulation::seeded(Config::default(), 10, 3.0, 42).unwrap();
    let mut second = Simulation::seeded(Config::default(), 10, 3.0, 42).unwrap();
    assert_eq!(first.agents(), second.agents());
    for _ in 0..50 {
        first.step().unwrap();
        second.step().unwrap();
    }
    assert_eq!(first.agents(), second.agents());
}

#[test]
fn swept_collisions_terminate_participants_but_not_survivors() {
    let config = Config {
        dt: 1.0,
        speed: 10.0,
        ..inertial_config()
    };
    let mut world = Simulation::new(
        config,
        vec![
            agent([-1.0, 0.0], [1.0, 0.0]),
            agent([1.0, 0.0], [-1.0, 0.0]),
            agent([0.0, 3.0], [0.0, 1.0]),
        ],
    )
    .unwrap();
    world.step().unwrap();
    assert_eq!(world.agents()[0].state(), AgentState::Terminated);
    assert_eq!(world.agents()[1].state(), AgentState::Terminated);
    assert_eq!(world.agents()[0].velocity(), [0.0, 0.0]);
    close(world.agents()[0].position()[0], -config.body_radius);
    close(world.agents()[1].position()[0], config.body_radius);
    assert_eq!(world.agents()[2].state(), AgentState::Moving);
    close(world.agents()[2].position()[1], 13.0);
    world.set_command(Command::Move([1.0, 0.0])).unwrap();
    world.step().unwrap();
    assert_eq!(world.agents()[0].state(), AgentState::Terminated);
}

#[test]
fn a_terminated_agent_cannot_cause_a_later_collision() {
    let config = Config {
        dt: 1.0,
        speed: 2.0,
        ..inertial_config()
    };
    let mut world = Simulation::new(
        config,
        vec![
            agent([0.0, 0.0], [1.0, 0.0]),
            agent([1.0, 0.0], [-1.0, 0.0]),
            agent([2.0, 0.0], [-1.0, 0.0]),
        ],
    )
    .unwrap();
    world.step().unwrap();
    assert_eq!(world.agents()[2].state(), AgentState::Moving);
    close(world.agents()[2].position()[0], 0.0);
}

#[test]
fn simultaneous_three_agent_contact_terminates_every_participant() {
    let config = Config {
        dt: 2.0,
        ..inertial_config()
    };
    let positions = [
        [1.0, 0.0],
        [-0.5, 3.0_f64.sqrt() / 2.0],
        [-0.5, -3.0_f64.sqrt() / 2.0],
    ];
    let agents = positions
        .into_iter()
        .map(|p| agent(p, [-p[0], -p[1]]))
        .collect();
    let mut world = Simulation::new(config, agents).unwrap();
    world.step().unwrap();
    assert!(
        world
            .agents()
            .iter()
            .all(|a| a.state() == AgentState::Terminated)
    );
}

#[test]
fn arrival_stops_at_the_region_instead_of_overshooting() {
    let config = Config {
        dt: 1.0,
        speed: 10.0,
        ..inertial_config()
    };
    let mut world = Simulation::new(config, vec![agent([-2.0, 0.0], [1.0, 0.0])]).unwrap();
    world.set_command(Command::Hold).unwrap();
    world
        .set_command(Command::Gather {
            center: [0.0, 0.0],
            radius: 0.5,
        })
        .unwrap();
    for _ in 0..100 {
        world.step().unwrap();
    }
    assert!(world.agents()[0].position()[0].abs() <= 0.5 + 1e-10);
    assert_eq!(world.agents()[0].state(), AgentState::Arrived);
    assert_eq!(world.agents()[0].velocity(), [0.0, 0.0]);
    world.step().unwrap();
    assert!(world.agents()[0].position()[0].abs() <= 0.5 + 1e-10);
    world.set_command(Command::Move([1.0, 0.0])).unwrap();
    assert_eq!(world.agents()[0].state(), AgentState::Moving);
}

#[test]
fn a_fast_copter_brakes_instead_of_snapping_to_a_stop_at_the_target() {
    let config = Config::default();
    let mut world = Simulation::new(config, vec![agent([-0.2, 0.0], [1.0, 0.0])]).unwrap();
    world
        .set_command(Command::Gather {
            center: [0.0, 0.0],
            radius: 1.0,
        })
        .unwrap();
    let mut previous = world.agents()[0].velocity();
    for _ in 0..200 {
        world.step().unwrap();
        let velocity = world.agents()[0].velocity();
        assert!(
            (velocity[0] - previous[0]).hypot(velocity[1] - previous[1])
                <= config.max_acceleration.max(config.max_braking) * config.dt + 1e-10
        );
        previous = velocity;
    }
    assert_eq!(world.agents()[0].state(), AgentState::Arrived);
}

#[test]
fn invalid_inputs_are_rejected_without_panics() {
    assert!(Agent::new([f64::NAN, 0.0], [1.0, 0.0]).is_err());
    assert!(Agent::new([0.0, 0.0], [0.0, 0.0]).is_err());
    assert!(Agent::new([0.0, 0.0], [f64::INFINITY, 0.0]).is_err());
    for config in [
        Config {
            dt: 0.0,
            ..Config::default()
        },
        Config {
            speed: -1.0,
            ..Config::default()
        },
        Config {
            turn_response: 1.1,
            ..Config::default()
        },
        Config {
            separation_weight: f64::NAN,
            ..Config::default()
        },
        Config {
            max_acceleration: 0.0,
            ..Config::default()
        },
        Config {
            max_braking: -1.0,
            ..Config::default()
        },
        Config {
            max_jerk: f64::INFINITY,
            ..Config::default()
        },
    ] {
        assert!(Simulation::new(config, vec![agent([0.0, 0.0], [1.0, 0.0])]).is_err());
    }
    assert!(Simulation::seeded(Config::default(), 0, 1.0, 0).is_err());
    assert!(Simulation::seeded(Config::default(), 2, 0.001, 0).is_err());
    assert!(
        Simulation::new(
            Config::default(),
            vec![agent([0.0, 0.0], [1.0, 0.0]), agent([0.0, 0.0], [0.0, 1.0]),]
        )
        .is_err()
    );
}

#[test]
fn an_invalid_command_does_not_change_the_run() {
    let mut world =
        Simulation::new(inertial_config(), vec![agent([0.0, 0.0], [1.0, 0.0])]).unwrap();
    assert!(world.set_command(Command::Move([0.0, 0.0])).is_err());
    assert!(
        world
            .set_command(Command::Gather {
                center: [0.0, 0.0],
                radius: -1.0
            })
            .is_err()
    );
    world.step().unwrap();
    close(world.agents()[0].position()[0], 0.1);
}

#[test]
fn a_numerical_failure_does_not_partially_advance_the_run() {
    let config = Config {
        dt: 1.0,
        speed: f64::MAX,
        ..inertial_config()
    };
    let mut world = Simulation::new(config, vec![agent([f64::MAX, 0.0], [1.0, 0.0])]).unwrap();
    let before = world.agents().to_vec();
    assert!(world.step().is_err());
    assert_eq!(world.agents(), before);
    assert_eq!(world.steps(), 0);
}

#[test]
fn recreating_a_run_does_not_multiply_the_existing_speed() {
    let first = Simulation::new(
        Config {
            speed: 2.0,
            ..inertial_config()
        },
        vec![agent([0.0, 0.0], [1.0, 0.0])],
    )
    .unwrap();
    let second = Simulation::new(
        Config {
            speed: 3.0,
            ..inertial_config()
        },
        first.agents().to_vec(),
    )
    .unwrap();
    close(second.agents()[0].velocity()[0], 3.0);
}

#[test]
fn an_impossible_gather_preserves_the_previous_command_and_state() {
    let config = Config {
        dt: 1.0,
        speed: 2.0,
        ..inertial_config()
    };
    let mut world = Simulation::new(
        config,
        vec![
            agent([0.0, 0.0], [1.0, 0.0]),
            agent([-1.0, 0.0], [1.0, 0.0]),
        ],
    )
    .unwrap();
    let before = world.agents().to_vec();
    assert!(
        world
            .set_command(Command::Gather {
                center: [0.0, 0.0],
                radius: 0.0,
            })
            .is_err()
    );
    assert_eq!(world.agents(), before);
    assert_eq!(world.command(), Command::Flock);
}

#[test]
fn controlled_gather_hovers_then_migrates_without_losing_agents() {
    for seed in [0, 42, 99] {
        let mut world = Simulation::seeded(Config::default(), 10, 3.0, seed).unwrap();
        world.set_command(Command::Hold).unwrap();
        for center in [[3.0, 2.0], [-2.0, -3.0]] {
            world
                .set_command(Command::Gather {
                    center,
                    radius: 1.0,
                })
                .unwrap();
            for _ in 0..300 {
                world.step().unwrap();
                for (i, a) in world.agents().iter().enumerate() {
                    assert_ne!(a.state(), AgentState::Terminated);
                    assert!(a.velocity()[0].hypot(a.velocity()[1]) <= 1.0 + 1e-10);
                    for b in &world.agents()[..i] {
                        assert!(
                            (a.position()[0] - b.position()[0])
                                .hypot(a.position()[1] - b.position()[1])
                                > 0.05
                        );
                    }
                }
            }
            assert!(
                world
                    .agents()
                    .iter()
                    .all(|a| a.state() == AgentState::Arrived)
            );
            assert!(world.agents().iter().all(|a| {
                (a.position()[0] - center[0]).hypot(a.position()[1] - center[1])
                    <= 1.0 + Config::default().neighbor_radius + 1e-10
            }));
            let mean = [
                world.agents().iter().map(|a| a.position()[0]).sum::<f64>() / 10.0,
                world.agents().iter().map(|a| a.position()[1]).sum::<f64>() / 10.0,
            ];
            assert!(
                (mean[0] - center[0]).hypot(mean[1] - center[1]) <= 1.0,
                "centroid={mean:?}, center={center:?}"
            );
            let parked = world.agents().to_vec();
            for _ in 0..20 {
                world.step().unwrap();
            }
            assert_eq!(world.agents(), parked);
        }
    }
}

#[test]
fn live_retargeting_spreading_and_hover_preserve_living_copters() {
    let mut world = Simulation::seeded(Config::default(), 10, 3.0, 42).unwrap();
    world.set_command(Command::Hold).unwrap();
    let initial = world.agents().to_vec();
    world.step().unwrap();
    assert_eq!(world.agents(), initial);
    // Replanning on every tick models a held mouse button with a moving target.
    for tick in 0..30 {
        world
            .set_command(Command::Gather {
                center: [5.0, tick as f64 / 30.0],
                radius: 1.0,
            })
            .unwrap();
        world.step().unwrap();
    }
    let mean_x =
        |world: &Simulation| world.agents().iter().map(|a| a.position()[0]).sum::<f64>() / 10.0;
    assert!(mean_x(&world) > 1.0);
    world.set_command(Command::Spread { spacing: 0.8 }).unwrap();
    for _ in 0..200 {
        world.step().unwrap();
    }
    assert!(
        world
            .agents()
            .iter()
            .all(|a| a.state() == AgentState::Arrived)
    );
    for (i, agent) in world.agents().iter().enumerate() {
        for other in &world.agents()[i + 1..] {
            assert!(
                (agent.position()[0] - other.position()[0])
                    .hypot(agent.position()[1] - other.position()[1])
                    >= 0.8 * 0.98
            );
        }
    }
    world.set_command(Command::Hold).unwrap();
    let parked = world.agents().to_vec();
    for _ in 0..20 {
        world.step().unwrap();
    }
    assert_eq!(world.agents(), parked);
    world.set_command(Command::Move([-1.0, 0.0])).unwrap();
    world.step().unwrap();
    assert!(
        world
            .agents()
            .iter()
            .all(|a| a.state() == AgentState::Moving)
    );
}

#[test]
fn rejected_retarget_keeps_the_in_progress_maneuver() {
    let mut a = Simulation::seeded(Config::default(), 10, 3.0, 42).unwrap();
    let mut b = Simulation::seeded(Config::default(), 10, 3.0, 42).unwrap();
    for world in [&mut a, &mut b] {
        world
            .set_command(Command::Gather {
                center: [4.0, 3.0],
                radius: 1.0,
            })
            .unwrap();
        world.step().unwrap();
    }
    for command in [
        Command::Gather {
            center: [f64::INFINITY, 0.0],
            radius: 1.0,
        },
        Command::Gather {
            center: [0.0, 0.0],
            radius: -0.02,
        },
        Command::Spread { spacing: -0.1 },
    ] {
        assert!(a.set_command(command).is_err());
    }
    for _ in 0..100 {
        a.step().unwrap();
        b.step().unwrap();
    }
    assert_eq!(a.agents(), b.agents());
    assert_eq!(a.command(), b.command());
}

#[test]
fn dead_agents_stay_dead_when_survivors_receive_new_commands() {
    let mut world = Simulation::new(
        Config {
            speed: 10.0,
            dt: 1.0,
            ..inertial_config()
        },
        vec![
            agent([-1.0, 0.0], [1.0, 0.0]),
            agent([1.0, 0.0], [-1.0, 0.0]),
            agent([0.0, 3.0], [0.0, 1.0]),
        ],
    )
    .unwrap();
    world.step().unwrap();
    let dead = world.agents()[..2].to_vec();
    world
        .set_command(Command::Gather {
            center: [2.0, 0.0],
            radius: 1.0,
        })
        .unwrap();
    for _ in 0..100 {
        world.step().unwrap();
    }
    assert_eq!(&world.agents()[..2], dead);
    assert_eq!(world.agents()[2].state(), AgentState::Arrived);
}

#[test]
fn visible_danger_reduces_forward_speed_and_triggers_a_local_sidestep() {
    let make = |view| {
        let mut world = Simulation::new(
            Config {
                neighbor_radius: view,
                separation_radius: 0.1,
                ..Config::default()
            },
            vec![
                agent([0.0, 0.0], [1.0, 0.0]),
                agent([0.7, 0.0], [-1.0, 0.0]),
            ],
        )
        .unwrap();
        world
            .set_command(Command::Gather {
                center: [3.0, 0.0],
                radius: 1.0,
            })
            .unwrap();
        world.step().unwrap();
        world
    };
    let visible = make(1.0);
    let blind = make(0.2);
    let visible_v = visible.agents()[0].velocity();
    close(blind.agents()[0].velocity()[0], 1.0);
    assert!(visible_v[0] < blind.agents()[0].velocity()[0]);
    assert!(visible_v[1].abs() > 0.0);
    assert!(visible_v[0].hypot(visible_v[1]) < 1.0);
}

#[test]
fn an_unseen_neighbor_does_not_change_local_velocity() {
    let config = Config {
        neighbor_radius: 0.2,
        separation_radius: 0.1,
        ..Config::default()
    };
    let mut a = Simulation::new(config, vec![agent([0.0, 0.0], [1.0, 0.0])]).unwrap();
    a.set_command(Command::Gather {
        center: [5.0, 0.0],
        radius: 10.0,
    })
    .unwrap();
    let mut b = Simulation::new(
        config,
        vec![
            agent([0.0, 0.0], [1.0, 0.0]),
            agent([2.0, 0.0], [-1.0, 0.0]),
        ],
    )
    .unwrap();
    b.set_command(Command::Gather {
        center: [6.0, 0.0],
        radius: 10.0,
    })
    .unwrap();
    a.step().unwrap();
    b.step().unwrap();
    assert_eq!(a.agents()[0].velocity(), b.agents()[0].velocity());
}
