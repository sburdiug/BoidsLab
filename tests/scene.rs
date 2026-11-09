use rust_playground::simulation::{
    Agent, AgentState, Command, Config, NoiseZone, Rectangle, Route, Simulation, Square, WindZone,
};

fn copters(config: Config, positions: &[[f64; 2]]) -> Simulation {
    let mut world = Simulation::new(
        config,
        positions
            .iter()
            .map(|p| Agent::new(*p, [1.0, 0.0]).unwrap())
            .collect(),
    )
    .unwrap();
    world.set_command(Command::Hold).unwrap();
    world
}

#[test]
fn a_copter_detours_around_a_square_and_reaches_the_target_area() {
    let mut world = copters(Config::default(), &[[-3.0, 0.0]]);
    world
        .set_obstacle(Some(Square::new([0.0, 0.0], 2.0).unwrap().into()))
        .unwrap();
    world
        .set_command(Command::Gather {
            center: [3.0, 0.0],
            radius: 0.5,
        })
        .unwrap();
    let mut detour = 0.0_f64;
    for _ in 0..600 {
        world.step().unwrap();
        let a = world.agents()[0];
        assert_ne!(a.state(), AgentState::Terminated, "{a:?}");
        detour = detour.max(a.position()[1].abs());
    }
    assert!(detour > 1.0);
    assert_eq!(
        world.agents()[0].state(),
        AgentState::Arrived,
        "{:?}",
        world.agents()
    );
    let p = world.agents()[0].position();
    assert!((p[0] - 3.0).hypot(p[1]) <= 0.5, "{p:?}");
}

#[test]
fn swept_obstacle_contact_cannot_tunnel_and_rounded_corners_are_exact() {
    let config = Config {
        dt: 1.0,
        speed: 10.0,
        obstacle_weight: 0.0,
        ..Config::default()
    };
    let mut world = copters(config, &[[-3.0, 0.0]]);
    world
        .set_obstacle(Some(Square::new([0.0, 0.0], 2.0).unwrap().into()))
        .unwrap();
    world.set_command(Command::Move([1.0, 0.0])).unwrap();
    world.step().unwrap();
    assert_eq!(world.agents()[0].state(), AgentState::Terminated);
    assert!((world.agents()[0].position()[0] + 1.025).abs() < 1e-9);

    let mut world = copters(
        Config {
            dt: 1.0,
            speed: 0.05,
            obstacle_weight: 0.0,
            ..Config::default()
        },
        &[[-1.04, -1.01]],
    );
    world
        .set_obstacle(Some(Square::new([0.0, 0.0], 2.0).unwrap().into()))
        .unwrap();
    world.set_command(Command::Move([1.0, -1.0])).unwrap();
    world.step().unwrap();
    assert_eq!(world.agents()[0].state(), AgentState::Moving);
}

#[test]
fn steady_wind_is_compensated_during_hover_and_strong_wind_can_overpower_it() {
    for wind in [0.3, 2.0] {
        let mut world = copters(Config::default(), &[[0.0, 0.0]]);
        world.add_wind(
            WindZone::new(Square::new([0.0, 0.0], 1000.0).unwrap(), [0.0, wind]).unwrap(),
        );
        for _ in 0..150 {
            world.step().unwrap();
        }
        let a = world.agents()[0];
        assert!(a.air_velocity()[0].hypot(a.air_velocity()[1]) <= 1.0 + 1e-9);
        if wind < 1.0 {
            assert!(a.position()[1].abs() < 0.2, "{a:?}");
            assert!(a.velocity()[1].abs() < 0.01, "{a:?}");
        } else {
            assert!(a.position()[1] > 10.0, "{a:?}");
        }
    }
}

#[test]
fn noise_is_repeatable_read_only_and_does_not_move_the_true_position() {
    let mut world = copters(Config::default(), &[[0.0, 0.0], [4.0, 0.0]]);
    world.add_noise(NoiseZone::new(Square::new([0.0, 0.0], 2.0).unwrap(), 0.1).unwrap());
    let original = world.agents().to_vec();
    let measured = world.measured_position(0).unwrap();
    assert_ne!(measured, [0.0, 0.0]);
    assert_eq!(world.measured_position(0).unwrap(), measured);
    assert_eq!(world.measured_position(1).unwrap(), [4.0, 0.0]);
    assert_eq!(world.agents(), original);
}

#[test]
fn a_polyline_advances_independently_and_finishes_after_a_detour() {
    let mut world = copters(Config::default(), &[[-3.0, 0.0], [-4.0, 0.0]]);
    world
        .set_obstacle(Some(Square::new([0.0, 0.0], 2.0).unwrap().into()))
        .unwrap();
    world
        .set_route(Route::new(vec![[-3.0, 0.0], [3.0, 0.0], [3.0, 3.0]], 0.4).unwrap())
        .unwrap();
    world.step().unwrap();
    assert_eq!(world.agents()[0].waypoint_index(), 1);
    assert_eq!(world.agents()[1].waypoint_index(), 0);
    for _ in 0..800 {
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
    assert!(world.agents().iter().all(|a| a.waypoint_index() == 2));
}

#[test]
fn scene_edits_reject_bad_geometry_without_replacing_the_scene_or_command() {
    assert!(Square::new([0.0, 0.0], 0.0).is_err());
    assert!(Square::new([f64::INFINITY, 0.0], 1.0).is_err());
    let mut world = copters(Config::default(), &[[-3.0, 0.0]]);
    let square = Square::new([0.0, 0.0], 2.0).unwrap();
    world.set_obstacle(Some(square.into())).unwrap();
    assert!(
        world
            .set_obstacle(Some(Square::new([-3.0, 0.0], 1.0).unwrap().into()))
            .is_err()
    );
    assert_eq!(world.obstacle(), Some(square.into()));
    assert!(
        world
            .set_command(Command::Gather {
                center: [0.0, 0.0],
                radius: 1.0
            })
            .is_err()
    );
    assert_eq!(world.command(), Command::Hold);
}

#[test]
fn final_route_parking_can_expand_beyond_a_tiny_radius() {
    let mut world = copters(Config::default(), &[[-1.0, 0.0], [0.0, 0.0], [1.0, 0.0]]);
    world
        .set_route(Route::new(vec![[0.0, 0.0]], 0.05).unwrap())
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
}

#[test]
fn wind_driven_swept_contacts_are_not_limited_to_the_airspeed_reach() {
    let mut world = copters(
        Config {
            dt: 1.0,
            separation_weight: 0.0,
            ..Config::default()
        },
        &[[-2.0, 0.0], [2.0, 0.0]],
    );
    world.add_wind(WindZone::new(Square::new([-2.0, 0.0], 1.0).unwrap(), [10.0, 0.0]).unwrap());
    world.step().unwrap();
    assert!(
        world
            .agents()
            .iter()
            .all(|a| a.state() == AgentState::Terminated)
    );
    assert!((world.agents()[0].position()[0] - 1.95).abs() < 1e-9);
}

#[test]
fn noise_changes_control_and_hover_recovers_after_removing_wind() {
    let mut noisy = copters(Config::default(), &[[0.0, 0.0]]);
    noisy.add_noise(NoiseZone::new(Square::new([0.0, 0.0], 10.0).unwrap(), 0.03).unwrap());
    for _ in 0..10 {
        noisy.step().unwrap();
    }
    assert_ne!(noisy.agents()[0].position(), [0.0, 0.0]);

    let mut windy = copters(Config::default(), &[[0.0, 0.0]]);
    windy.add_wind(WindZone::new(Square::new([0.0, 0.0], 10.0).unwrap(), [0.3, 0.0]).unwrap());
    for _ in 0..100 {
        windy.step().unwrap();
    }
    windy.clear_fields();
    for _ in 0..100 {
        windy.step().unwrap();
    }
    let agent = windy.agents()[0];
    assert!(agent.position()[0].abs() < 0.02, "{agent:?}");
    assert!(agent.velocity()[0].abs() < 0.001, "{agent:?}");
}

#[test]
fn wind_just_above_the_airspeed_limit_cannot_trigger_an_impossible_hover() {
    let mut world = copters(Config::default(), &[[0.0, 0.0]]);
    world.add_wind(WindZone::new(Square::new([0.0, 0.0], 100.0).unwrap(), [1.0005, 0.0]).unwrap());
    for _ in 0..1500 {
        world.step().unwrap();
    }
    let a = world.agents()[0];
    assert!(
        a.air_velocity()[0].hypot(a.air_velocity()[1]) <= 1.0 + 1e-9,
        "{a:?}"
    );
    assert_eq!(a.state(), AgentState::Moving);
}

#[test]
fn disabled_centering_keeps_parallel_rows_in_a_wide_corridor() {
    for x in [-3.0, 0.0] {
        let mut world = copters(
            Config {
                route_centering: 0.0,
                route_cohesion: 0.0,
                ..Config::default()
            },
            &[[x, 0.7], [x, -0.7]],
        );
        world
            .set_route(Route::new(vec![[0.0, 0.0], [20.0, 0.0]], 1.0).unwrap())
            .unwrap();
        for _ in 0..60 {
            world.step().unwrap();
        }
        let agents = world.agents();
        assert!(agents.iter().all(|a| a.position()[0] > x + 3.0));
        assert!((agents[0].position()[1] - 0.7).abs() < 0.02, "{agents:?}");
        assert!((agents[1].position()[1] + 0.7).abs() < 0.02, "{agents:?}");
    }
}

#[test]
fn a_corridor_turn_carries_rows_into_the_next_segment() {
    let mut world = copters(
        Config {
            route_centering: 0.0,
            route_cohesion: 0.0,
            ..Config::default()
        },
        &[[0.0, 0.7], [0.0, -0.7]],
    );
    world
        .set_route(Route::new(vec![[0.0, 0.0], [5.0, 0.0], [5.0, 20.0]], 1.0).unwrap())
        .unwrap();
    for _ in 0..140 {
        world.step().unwrap();
    }
    let agents = world.agents();
    assert!(
        agents.iter().all(|a| a.state() == AgentState::Moving
            && a.waypoint_index() == 2
            && a.position()[1] > 4.0),
        "{agents:?}"
    );
    assert!(
        (agents[0].position()[0] - agents[1].position()[0]).abs() > 0.6,
        "{agents:?}"
    );
}

#[test]
fn a_displaced_copter_returns_towards_the_center_after_wind() {
    let mut world = copters(Config::default(), &[[0.0, 0.7]]);
    world
        .set_route(Route::new(vec![[0.0, 0.0], [50.0, 0.0]], 1.0).unwrap())
        .unwrap();
    world.step().unwrap();
    world.add_wind(WindZone::new(Square::new([0.0, 0.0], 200.0).unwrap(), [0.0, 2.0]).unwrap());
    for _ in 0..20 {
        world.step().unwrap();
    }
    assert!(world.agents()[0].position()[1] > 1.5);
    world.clear_fields();
    for _ in 0..100 {
        world.step().unwrap();
    }
    let a = world.agents()[0];
    assert_eq!(a.state(), AgentState::Moving);
    assert!(
        a.position()[0] > 5.0 && a.position()[1].abs() < 0.3,
        "{a:?}"
    );
}

#[test]
fn a_rotated_square_has_exact_contacts_and_can_be_detoured() {
    let obstacle = Square::new([0.0, 0.0], 2.0)
        .unwrap()
        .rotated(std::f64::consts::FRAC_PI_4)
        .unwrap();
    let mut fast = copters(
        Config {
            dt: 1.0,
            speed: 10.0,
            obstacle_weight: 0.0,
            ..Config::default()
        },
        &[[-3.0, 0.0]],
    );
    fast.set_obstacle(Some(obstacle.into())).unwrap();
    fast.set_command(Command::Move([1.0, 0.0])).unwrap();
    fast.step().unwrap();
    assert_eq!(fast.agents()[0].state(), AgentState::Terminated);
    assert!((fast.agents()[0].position()[0] + 2.0_f64.sqrt() + 0.025).abs() < 1e-9);

    let mut world = copters(Config::default(), &[[-3.0, 0.0]]);
    world.set_obstacle(Some(obstacle.into())).unwrap();
    world
        .set_command(Command::Gather {
            center: [3.0, 0.0],
            radius: 0.5,
        })
        .unwrap();
    for _ in 0..600 {
        world.step().unwrap();
    }
    assert_eq!(
        world.agents()[0].state(),
        AgentState::Arrived,
        "{:?}",
        world.agents()
    );
}

#[test]
fn rotating_an_obstacle_into_a_living_agent_is_rejected_atomically() {
    let mut world = copters(Config::default(), &[[1.2, 0.0]]);
    let square = Square::new([0.0, 0.0], 2.0).unwrap();
    world.set_obstacle(Some(square.into())).unwrap();
    assert!(
        world
            .set_obstacle(Some(
                square.rotated(std::f64::consts::FRAC_PI_4).unwrap().into()
            ))
            .is_err()
    );
    assert_eq!(world.obstacle(), Some(square.into()));
    assert!(square.rotated(f64::NAN).is_err());
}

#[test]
fn rectangular_wind_and_noise_rotate_resize_and_keep_strength_independent() {
    let angle = std::f64::consts::FRAC_PI_2;
    let area = Rectangle::new([0.0, 0.0], 2.0, 6.0)
        .unwrap()
        .rotated(angle)
        .unwrap();
    assert!(area.contains([0.0, 2.5]));
    assert!(!area.contains([2.5, 0.0]));
    let mut world = copters(Config::default(), &[[0.0, 2.5], [2.5, 0.0]]);
    let wind = WindZone::rectangular([0.0, 0.0], 2.0, 6.0, angle, 0.3).unwrap();
    world.add_wind(wind);
    world.add_noise(NoiseZone::new(area, 0.1).unwrap());
    assert_ne!(world.measured_position(0).unwrap(), [0.0, 2.5]);
    assert_eq!(world.measured_position(1).unwrap(), [2.5, 0.0]);
    world.step().unwrap();
    assert!((world.agents()[0].wind_velocity()[1] - 0.3).abs() < 1e-12);
    assert_eq!(world.agents()[1].wind_velocity(), [0.0, 0.0]);
    world.replace_wind(0, wind.rotated(0.0).unwrap()).unwrap();
    world
        .replace_noise(
            0,
            NoiseZone::new(Rectangle::new([0.0, 0.0], 2.0, 8.0).unwrap(), 0.2).unwrap(),
        )
        .unwrap();
    world.step().unwrap();
    assert_eq!(world.agents()[0].wind_velocity(), [0.0, 0.0]);
    assert!((world.agents()[1].wind_velocity()[0] - 0.3).abs() < 1e-12);
    assert_eq!(world.noise_zones()[0].area().length(), 8.0);
}
