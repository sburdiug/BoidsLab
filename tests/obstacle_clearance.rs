use rust_playground::simulation::{Agent, AgentState, Command, Config, Route, Simulation, Square};

#[test]
fn copters_keep_moving_along_a_face_when_inside_the_planning_gap() {
    for angle in [0.0, 0.03, -0.03] {
        for count in [1, 2] {
            let mut world = Simulation::new(
                Config::default(),
                (0..count)
                    .map(|i| Agent::new([1.11, -0.6 + i as f64 * 0.45], [0.0, 1.0]).unwrap())
                    .collect(),
            )
            .unwrap();
            world.set_command(Command::Hold).unwrap();
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
                .set_route(Route::new(vec![[1.2, -0.6], [0.0, 4.0], [0.0, 8.0]], 1.0).unwrap())
                .unwrap();
            let mut slow_ticks = vec![0; count];
            for step in 0..100 {
                world.step().unwrap();
                for (i, a) in world.agents().iter().enumerate() {
                    assert_ne!(a.state(), AgentState::Terminated, "{a:?}");
                    if step > 10 && a.position()[1] < 0.8 && a.velocity()[1] < 0.1 {
                        slow_ticks[i] += 1;
                    }
                }
            }
            assert!(
                slow_ticks.iter().all(|&n| n <= 2),
                "angle={angle} n={count} stopped={slow_ticks:?}"
            );
            assert!(
                world.agents().iter().all(|a| a.position()[1] > 3.0),
                "{:?}",
                world.agents()
            );
        }
    }
}

#[test]
fn wall_perception_uses_the_nearest_surface_and_the_agents_view_radius() {
    use rust_playground::simulation::Rectangle;
    for angle in [0.0_f64, std::f64::consts::FRAC_PI_2] {
        let rotate = |p: [f64; 2]| {
            [
                p[0] * angle.cos() - p[1] * angle.sin(),
                p[0] * angle.sin() + p[1] * angle.cos(),
            ]
        };
        for copter in [false, true] {
            let run = |surface: Option<f64>| {
                let mut world = Simulation::new(
                    Config {
                        turn_response: 1.0,
                        ..Config::default()
                    },
                    vec![Agent::new([0.0; 2], rotate([1.0, 0.0])).unwrap()],
                )
                .unwrap();
                if let Some(distance) = surface {
                    world
                        .set_obstacle(Some(
                            Rectangle::new(rotate([distance + 2.0, 0.0]), 2.0, 4.0)
                                .unwrap()
                                .rotated(angle)
                                .unwrap(),
                        ))
                        .unwrap();
                }
                if copter {
                    world.set_command(Command::Hold).unwrap();
                    world
                        .set_command(Command::Gather {
                            center: rotate([6.0, 0.0]),
                            radius: 0.1,
                        })
                        .unwrap();
                } else {
                    world
                        .set_command(Command::Move(rotate([1.0, 0.0])))
                        .unwrap();
                }
                world.step().unwrap();
                assert_eq!(world.agents()[0].state(), AgentState::Moving);
                world.agents()[0].velocity()
            };
            let direct = run(None);
            assert_eq!(run(Some(1.001)), direct, "wall outside view changed motion");
            assert_ne!(
                run(Some(0.999)),
                direct,
                "visible surface was ignored: copter={copter}, angle={angle}"
            );
        }
    }
}
