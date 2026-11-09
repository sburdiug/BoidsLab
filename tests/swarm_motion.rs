use rust_playground::simulation::{Agent, AgentState, Command, Config, Route, Simulation, Square};

fn copters(config: Config, positions: &[[f64; 2]]) -> Simulation {
    let agents = positions
        .iter()
        .map(|&p| Agent::new(p, [1.0, 0.0]).unwrap())
        .collect();
    let mut world = Simulation::new(config, agents).unwrap();
    world.set_command(Command::Hold).unwrap();
    world
}
fn advance(world: &mut Simulation, steps: usize) {
    for _ in 0..steps {
        world.step().unwrap();
    }
}
fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

#[test]
fn route_centering_releases_the_edge_and_recenters_after_a_turn() {
    let mut world = copters(Config::default(), &[[0.0, 0.9]]);
    world
        .set_route(Route::new(vec![[0.0, 0.0], [6.0, 0.0], [6.0, 25.0]], 1.0).unwrap())
        .unwrap();
    advance(&mut world, 55);
    assert!(
        world.agents()[0].position()[1].abs() < 0.3,
        "{:?}",
        world.agents()
    );
    advance(&mut world, 120);
    let a = world.agents()[0];
    assert_eq!(a.state(), AgentState::Moving);
    assert!(
        a.position()[1] > 5.0 && (a.position()[0] - 6.0).abs() < 0.3,
        "{a:?}"
    );
}

#[test]
fn route_cohesion_reduces_longitudinal_stretch_without_stopping_the_swarm() {
    let positions: Vec<_> = [-0.6, 0.0, 0.6]
        .into_iter()
        .flat_map(|x| [-0.5, 0.0, 0.5].map(|y| [x, y]))
        .collect();
    let run = |cohesion| {
        let mut world = copters(
            Config {
                neighbor_radius: 3.0,
                route_cohesion: cohesion,
                ..Config::default()
            },
            &positions,
        );
        world
            .set_route(Route::new(vec![[0.0, 0.0], [50.0, 0.0]], 2.0).unwrap())
            .unwrap();
        advance(&mut world, 150);
        assert!(
            world
                .agents()
                .iter()
                .all(|a| a.state() == AgentState::Moving),
            "{:?}",
            world.agents()
        );
        let xs: Vec<_> = world.agents().iter().map(|a| a.position()[0]).collect();
        assert!(xs.iter().sum::<f64>() / xs.len() as f64 > 3.0);
        xs.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            - xs.iter().copied().fold(f64::INFINITY, f64::min)
    };
    let independent = run(0.0);
    let swarm = run(1.0);
    assert!(
        swarm < independent * 0.8,
        "independent={independent}, swarm={swarm}"
    );
}

#[test]
fn spread_sets_local_spacing_instead_of_a_ring_and_keeps_isolated_agents_in_place() {
    let positions: Vec<_> = [-0.2, 0.0, 0.2]
        .into_iter()
        .flat_map(|x| [-0.2, 0.0, 0.2].map(|y| [x, y]))
        .chain([[10.0, 3.0]])
        .collect();
    let mut world = copters(
        Config {
            neighbor_radius: 2.0,
            ..Config::default()
        },
        &positions,
    );
    world.set_command(Command::Spread { spacing: 0.7 }).unwrap();
    advance(&mut world, 600);
    assert!(
        world
            .agents()
            .iter()
            .all(|a| a.state() == AgentState::Arrived),
        "{:?}",
        world.agents()
    );
    for (i, a) in world.agents().iter().enumerate() {
        for b in &world.agents()[i + 1..] {
            assert!(
                distance(a.position(), b.position()) >= 0.7 * 0.98,
                "{:?}",
                world.agents()
            );
        }
    }
    assert_eq!(world.agents()[9].position(), [10.0, 3.0]);
    assert!(distance(world.agents()[4].position(), [0.0, 0.0]) < 0.05);
}

#[test]
fn unobservable_or_unsafe_spread_gaps_are_rejected_atomically() {
    let mut world = copters(Config::default(), &[[0.0, 0.0], [0.2, 0.0]]);
    let before = world.agents().to_vec();
    for spacing in [0.0, 0.01, 1.001, f64::NAN] {
        assert!(world.set_command(Command::Spread { spacing }).is_err());
        assert_eq!(world.command(), Command::Hold);
        assert_eq!(world.agents(), before);
    }
    world.set_command(Command::Spread { spacing: 1.0 }).unwrap();
}

#[test]
fn obstacle_contact_still_kills_when_perception_is_too_short_to_brake() {
    let mut world = Simulation::new(
        Config {
            speed: 4.0,
            dt: 0.5,
            neighbor_radius: 0.3,
            ..Config::default()
        },
        vec![Agent::new([-1.6, 0.0], [1.0, 0.0]).unwrap()],
    )
    .unwrap();
    world
        .set_obstacle(Some(Square::new([0.0, 0.0], 2.0).unwrap().into()))
        .unwrap();
    world
        .set_command(Command::Gather {
            center: [4.0, 0.0],
            radius: 0.5,
        })
        .unwrap();
    advance(&mut world, 2);
    assert_eq!(world.agents()[0].state(), AgentState::Terminated);
    assert!((world.agents()[0].position()[0] + 1.025).abs() < 1e-9);
}

#[test]
fn copters_clear_square_corners_with_and_without_extra_clearance() {
    for angle in [0.0, 0.49] {
        for obstacle_clearance in [0.0, 0.15] {
            let mut world = copters(
                Config {
                    obstacle_clearance,
                    ..Config::default()
                },
                &[[-3.0, 0.4]],
            );
            world
                .set_obstacle(Some(
                    Square::new([0.0, 0.0], 2.0)
                        .unwrap()
                        .rotated(angle)
                        .unwrap()
                        .into(),
                ))
                .unwrap();
            world
                .set_command(Command::Gather {
                    center: [3.0, -0.4],
                    radius: 0.5,
                })
                .unwrap();
            advance(&mut world, 1000);
            assert_eq!(
                world.agents()[0].state(),
                AgentState::Arrived,
                "angle={angle}, gap={obstacle_clearance}, {:?}",
                world.agents()
            );
            assert!(distance(world.agents()[0].position(), [3.0, -0.4]) <= 0.5);
        }
    }
}

#[test]
fn a_swarm_routes_around_a_square_instead_of_parking_against_its_face() {
    let positions: Vec<_> = [-3.8, -3.4, -3.0]
        .into_iter()
        .flat_map(|x| [-0.4, 0.0, 0.4].map(|y| [x, y]))
        .collect();
    let mut world = copters(Config::default(), &positions);
    world
        .set_obstacle(Some(Square::new([0.0, 0.0], 2.0).unwrap().into()))
        .unwrap();
    world
        .set_route(Route::new(vec![[-3.4, 0.0], [3.0, 0.0], [3.0, 8.0]], 1.0).unwrap())
        .unwrap();
    advance(&mut world, 1600);
    assert!(
        world
            .agents()
            .iter()
            .all(|a| a.state() == AgentState::Arrived && a.position()[1] > 6.0),
        "{:?}",
        world.agents()
    );
}
