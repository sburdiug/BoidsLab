use rust_playground::simulation::{
    AgentState, Command, Config, Interaction, Rectangle, Route, Simulation, SpawnConfig,
};
fn spawn(center: [f64; 2], count: usize, seed: u64) -> SpawnConfig {
    SpawnConfig {
        center,
        count,
        seed,
        radius: 0.001,
    }
}
#[test]
fn empty_world_accepts_groups_at_any_tick_and_preserves_ids_and_profiles() {
    let mut world = Simulation::empty(Config::default()).unwrap();
    world.step().unwrap();
    assert!(world.agents().is_empty());
    assert_eq!(world.steps(), 1);
    let a = world
        .add_group(Config::default(), spawn([2.0, 3.0], 1, 42), Command::Hold)
        .unwrap();
    let first = world.agents()[0];
    let b = world
        .add_group(Config::default(), spawn([-2.0, 0.0], 1, 99), Command::Flock)
        .unwrap();
    assert_eq!((a, b), (0, 1));
    assert_eq!(world.agents()[0], first);
    assert_eq!(world.agents()[1].group_id(), b);
    assert_eq!(
        world.group(a).unwrap().profile_hash(),
        world.group(b).unwrap().profile_hash()
    );
    assert_eq!(
        world.group(a).unwrap().color(),
        world.group(b).unwrap().color()
    );
    assert_eq!(world.group(a).unwrap().color()[3], 255);
    assert!(world.events().iter().all(|e| e.step == 1));
}
#[test]
fn directed_avoidance_does_not_copy_another_groups_flocking_behavior() {
    for mode in [Interaction::React, Interaction::Avoid, Interaction::Ignore] {
        let config = Config {
            turn_response: 1.0,
            neighbor_radius: 2.0,
            separation_radius: 1.0,
            separation_weight: 1.0,
            alignment_weight: 0.0,
            cohesion_weight: 10.0,
            command_weight: 0.0,
            ..Config::default()
        };
        let mut w = Simulation::empty(config).unwrap();
        let a = w
            .add_group(config, spawn([0.0, 0.0], 1, 1), Command::Hold)
            .unwrap();
        let b = w
            .add_group(config, spawn([0.5, 0.0], 1, 2), Command::Hold)
            .unwrap();
        w.set_interaction(b, a, mode).unwrap();
        assert_eq!(w.interaction(a, b).unwrap(), Interaction::React);
        assert_eq!(w.perceives(1, 0), mode != Interaction::Ignore);
        w.set_group_command(a, Command::Flock).unwrap();
        w.set_group_command(b, Command::Flock).unwrap();
        w.step().unwrap();
        if mode == Interaction::React {
            assert!(w.agents()[1].velocity()[0] < 0.0);
        } else {
            assert!(w.agents()[1].velocity()[0] > 0.0);
        }
    }
}
#[test]
fn ignored_groups_still_have_swept_contacts_with_their_own_body_radii() {
    let cfg = Config {
        dt: 0.5,
        turn_response: 1.0,
        separation_weight: 0.0,
        alignment_weight: 0.0,
        cohesion_weight: 0.0,
        ..Config::default()
    };
    let mut w = Simulation::empty(cfg).unwrap();
    let a = w
        .add_group(
            Config {
                speed: 2.0,
                body_radius: 0.1,
                ..cfg
            },
            spawn([-0.5, 0.0], 1, 1),
            Command::Hold,
        )
        .unwrap();
    let b = w
        .add_group(
            Config {
                speed: 0.5,
                body_radius: 0.2,
                ..cfg
            },
            spawn([0.5, 0.0], 1, 2),
            Command::Hold,
        )
        .unwrap();
    w.set_interaction(a, b, Interaction::Ignore).unwrap();
    w.set_interaction(b, a, Interaction::Ignore).unwrap();
    w.set_group_command(a, Command::Move([1.0, 0.0])).unwrap();
    w.set_group_command(b, Command::Move([-1.0, 0.0])).unwrap();
    w.step().unwrap();
    assert!(
        w.agents()
            .iter()
            .all(|a| a.state() == AgentState::Terminated)
    );
    let p = w.agents()[0].position();
    let q = w.agents()[1].position();
    assert!(((p[0] - q[0]).hypot(p[1] - q[1]) - 0.3).abs() < 1e-9);
}
#[test]
fn group_commands_profiles_and_paths_are_independent_and_replayable_from_seed() {
    let mut w = Simulation::empty(Config::default()).unwrap();
    let path = w.add_path(Route::new(vec![[0.0, 0.0], [4.0, 0.0]], 0.5).unwrap());
    let a = w
        .add_group(Config::default(), spawn([0.0, 0.0], 1, 42), Command::Hold)
        .unwrap();
    let b = w
        .add_group(Config::default(), spawn([0.0, 4.0], 1, 99), Command::Hold)
        .unwrap();
    let other = w.agents()[1];
    w.assign_path(a, path).unwrap();
    for _ in 0..20 {
        w.step().unwrap();
    }
    assert!(w.agents()[0].position()[0] > 0.5);
    assert_eq!(w.agents()[1], other);
    let changed = Config {
        speed: 0.5,
        ..w.group(b).unwrap().config()
    };
    w.set_group_config(b, changed).unwrap();
    assert_ne!(w.group(a).unwrap().color(), w.group(b).unwrap().color());
    assert_ne!(
        w.group(a).unwrap().profile_hash(),
        w.group(b).unwrap().profile_hash()
    );
    let restart = w.restarted().unwrap();
    assert_eq!(restart.steps(), 0);
    assert_eq!(restart.paths(), w.paths());
    assert_eq!(restart.group(a).unwrap().command(), Command::FollowRoute);
    assert_eq!(restart.group(b).unwrap().config().speed, 0.5);
}
#[test]
fn failed_spawns_and_invalid_edits_leave_world_and_history_unchanged() {
    let mut w = Simulation::empty(Config::default()).unwrap();
    let a = w
        .add_group(Config::default(), spawn([0.0, 0.0], 1, 42), Command::Hold)
        .unwrap();
    let before = w.agents().to_vec();
    let events = w.events().len();
    assert!(
        w.add_group(Config::default(), spawn([0.0, 0.0], 2, 99), Command::Flock)
            .is_err()
    );
    assert!(
        w.add_group(
            Config {
                dt: 0.2,
                ..Config::default()
            },
            spawn([3.0, 0.0], 1, 99),
            Command::Hold
        )
        .is_err()
    );
    assert!(
        w.set_group_config(
            a,
            Config {
                speed: f64::NAN,
                ..Config::default()
            }
        )
        .is_err()
    );
    assert!(w.set_interaction(a, 999, Interaction::Ignore).is_err());
    assert_eq!(w.agents(), before);
    assert_eq!(w.events().len(), events);
    assert_eq!(w.groups().len(), 1);
}
#[test]
fn rectangular_obstacles_keep_exact_swept_contacts() {
    let cfg = Config {
        speed: 10.0,
        dt: 1.0,
        obstacle_weight: 0.0,
        ..Config::default()
    };
    let mut w = Simulation::empty(cfg).unwrap();
    w.set_obstacle(Some(Rectangle::new([0.0, 0.0], 1.0, 4.0).unwrap()))
        .unwrap();
    let id = w
        .add_group(cfg, spawn([-3.0, 0.0], 1, 42), Command::Hold)
        .unwrap();
    w.set_group_command(id, Command::Move([1.0, 0.0])).unwrap();
    w.step().unwrap();
    assert_eq!(w.agents()[0].state(), AgentState::Terminated);
    assert!((w.agents()[0].position()[0] + 2.025).abs() < 1e-9);
}

#[test]
fn gathering_uses_nearby_free_space_instead_of_visiting_the_center() {
    let mut w = Simulation::new(
        Config::default(),
        vec![rust_playground::simulation::Agent::new([0.8, 0.0], [1.0, 0.0]).unwrap()],
    )
    .unwrap();
    w.set_command(Command::Hold).unwrap();
    let p = w.agents()[0].position();
    w.set_command(Command::Gather {
        center: [0.0, 0.0],
        radius: 1.0,
    })
    .unwrap();
    for _ in 0..50 {
        w.step().unwrap();
    }
    assert_eq!(w.agents()[0].position(), p);
    assert_eq!(w.agents()[0].state(), AgentState::Arrived);
}
#[test]
fn a_large_soft_region_spreads_a_dense_cluster_without_forcing_a_perimeter() {
    let cfg = Config {
        neighbor_radius: 3.0,
        ..Config::default()
    };
    let positions: Vec<_> = [-0.15, 0.0, 0.15]
        .into_iter()
        .flat_map(|x| [-0.15, 0.0, 0.15].map(|y| [x, y]))
        .collect();
    let mut w = Simulation::new(
        cfg,
        positions
            .iter()
            .map(|&p| rust_playground::simulation::Agent::new(p, [1.0, 0.0]).unwrap())
            .collect(),
    )
    .unwrap();
    w.set_command(Command::Hold).unwrap();
    w.set_command(Command::Gather {
        center: [0.0, 0.0],
        radius: 2.0,
    })
    .unwrap();
    for _ in 0..600 {
        w.step().unwrap();
    }
    assert!(
        w.agents().iter().all(|a| a.state() == AgentState::Arrived),
        "{:?}",
        w.agents()
    );
    let extent = w
        .agents()
        .iter()
        .map(|a| a.position()[0].hypot(a.position()[1]))
        .fold(0.0_f64, f64::max);
    assert!(extent > 1.0 && extent <= 2.025, "extent={extent}");
    assert!(
        w.agents()
            .iter()
            .any(|a| a.position()[0].hypot(a.position()[1]) < 0.5)
    );
}

#[test]
fn multiple_obstacles_are_preserved_and_all_stop_agents() {
    use rust_playground::simulation::Agent;
    let cfg = Config {
        speed: 10.0,
        dt: 1.0,
        obstacle_weight: 0.0,
        ..Config::default()
    };
    let mut w = Simulation::new(cfg, vec![Agent::new([-4.0, 3.0], [1.0, 0.0]).unwrap()]).unwrap();
    let first = Rectangle::new([0.0, 0.0], 1.0, 1.0).unwrap();
    let second = Rectangle::new([0.0, 3.0], 1.0, 1.0).unwrap();
    assert_eq!(w.add_obstacle(first).unwrap(), 0);
    assert_eq!(w.add_obstacle(second).unwrap(), 1);
    assert!(
        w.replace_obstacle(1, Rectangle::new([-4.0, 3.0], 1.0, 1.0).unwrap())
            .is_err()
    );
    assert_eq!(w.obstacles(), &[first, second]);
    w.step().unwrap();
    assert_eq!(w.agents()[0].state(), AgentState::Terminated);
    assert!((w.agents()[0].position()[0] + 0.525).abs() < 1e-9);

    let mut seeded = Simulation::empty(cfg).unwrap();
    seeded.add_obstacle(first).unwrap();
    seeded.add_obstacle(second).unwrap();
    seeded
        .add_group(cfg, spawn([-5.0, 0.0], 1, 1), Command::Flock)
        .unwrap();
    assert_eq!(seeded.restarted().unwrap().obstacles(), &[first, second]);
}

#[test]
fn controlled_groups_respect_directed_perception_while_contacts_remain_physical() {
    for (a_to_b, b_to_a) in [
        (Interaction::React, Interaction::React),
        (Interaction::Ignore, Interaction::React),
        (Interaction::Ignore, Interaction::Ignore),
    ] {
        let cfg = Config {
            speed: 4.0,
            dt: 0.1,
            ..Config::default()
        };
        let mut world = Simulation::empty(cfg).unwrap();
        let a = world
            .add_group(
                cfg,
                spawn([-0.2, 0.0], 1, 1),
                Command::Gather {
                    center: [4.0, 0.0],
                    radius: 0.1,
                },
            )
            .unwrap();
        let b = world
            .add_group(
                cfg,
                spawn([0.2, 0.0], 1, 2),
                Command::Gather {
                    center: [-4.0, 0.0],
                    radius: 0.1,
                },
            )
            .unwrap();
        world.set_interaction(a, b, a_to_b).unwrap();
        world.set_interaction(b, a, b_to_a).unwrap();
        world.step().unwrap();
        let dead = world
            .agents()
            .iter()
            .filter(|a| a.state() == AgentState::Terminated)
            .count();
        assert_eq!(
            dead,
            if a_to_b == Interaction::Ignore && b_to_a == Interaction::Ignore {
                2
            } else {
                0
            },
            "modes={a_to_b:?}/{b_to_a:?}: {:?}",
            world.agents()
        );
    }
}
