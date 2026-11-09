use rust_playground::simulation::{
    Command, Config, EventKind, NoiseZone, Rectangle, Route, ScenePreset, Simulation, SpawnConfig,
    WindZone,
};

fn scene() -> Simulation {
    let mut world = Simulation::empty(Config::default()).unwrap();
    world
        .add_obstacle(
            Rectangle::new([10.0, 0.0], 2.0, 4.0)
                .unwrap()
                .rotated(0.3)
                .unwrap(),
        )
        .unwrap();
    world
        .add_obstacle(Rectangle::new([-10.0, 0.0], 1.0, 3.0).unwrap())
        .unwrap();
    let area = Rectangle::new([4.0, 5.0], 1.0, 2.0)
        .unwrap()
        .rotated(0.7)
        .unwrap();
    world.add_wind(WindZone::new(area, [0.2, -0.5]).unwrap());
    world.add_noise(NoiseZone::new(area, 0.04).unwrap());
    world.add_path(Route::new(vec![[-2.0, 0.0], [3.0, 2.0]], 0.75).unwrap());
    world
}

#[test]
fn environment_roundtrip_can_be_applied_to_different_groups() {
    let original = scene();
    let preset = ScenePreset::capture(&original);
    let decoded = ScenePreset::parse(&preset.to_text()).unwrap();
    assert_eq!(decoded, preset);
    for (speed, center) in [(0.5, [0.0, 0.0]), (3.0, [0.0, -4.0])] {
        let config = Config {
            speed,
            ..Config::default()
        };
        let mut world = Simulation::empty(config).unwrap();
        let spawn = SpawnConfig {
            count: 2,
            center,
            radius: 0.5,
            seed: 7,
        };
        let group = world.add_group(config, spawn, Command::Flock).unwrap();
        world.step().unwrap();
        let agents = world.agents().to_vec();
        let step = world.steps();
        decoded.apply(&mut world).unwrap();
        assert_eq!(world.obstacles(), original.obstacles());
        assert_eq!(world.wind_zones(), original.wind_zones());
        assert_eq!(world.noise_zones(), original.noise_zones());
        assert_eq!(world.paths(), original.paths());
        assert_eq!(world.agents(), agents);
        assert_eq!(world.steps(), step);
        assert_eq!(world.group(group).unwrap().config().speed, speed);
        assert_eq!(world.group(group).unwrap().command(), Command::Flock);
        assert!(matches!(
            world.events().last().unwrap().kind,
            EventKind::PathsReplaced { .. }
        ));
    }
}

#[test]
fn invalid_and_conflicting_presets_are_rejected_without_partial_changes() {
    for text in [
        "",
        "SWARM_SCENE_V2\n",
        "SWARM_SCENE_V1\nnoise 0 0 1 1 0 NaN\n",
        "SWARM_SCENE_V1\npath 1 0\n",
        "SWARM_SCENE_V1\nunknown 0\n",
        "SWARM_SCENE_V1\nwind 0 0 1 1 0 1 0 5\n",
    ] {
        assert!(ScenePreset::parse(text).is_err(), "{text:?}");
    }
    let mut world = scene();
    world
        .add_group(
            Config::default(),
            SpawnConfig {
                count: 1,
                center: [0.0, 0.0],
                radius: 0.001,
                seed: 2,
            },
            Command::Gather {
                center: [2.0, 0.0],
                radius: 0.5,
            },
        )
        .unwrap();
    let previous = ScenePreset::capture(&world);
    let agents = world.agents().to_vec();
    let event_count = world.events().len();
    let blocked_body =
        ScenePreset::parse("SWARM_SCENE_V1\nobstacle 20 0 1 1 0\nobstacle 0 0 1 1 0\n").unwrap();
    assert!(blocked_body.apply(&mut world).is_err());
    let blocked_target = ScenePreset::parse("SWARM_SCENE_V1\nobstacle 2 0 1 1 0\n").unwrap();
    assert!(blocked_target.apply(&mut world).is_err());
    assert_eq!(ScenePreset::capture(&world), previous);
    assert_eq!(world.agents(), agents);
    assert_eq!(world.events().len(), event_count);
}

#[test]
fn loading_replaces_saved_paths_but_keeps_active_group_route() {
    let config = Config::default();
    let mut world = Simulation::empty(config).unwrap();
    let group = world
        .add_group(
            config,
            SpawnConfig {
                count: 1,
                center: [0.0, 0.0],
                radius: 0.001,
                seed: 8,
            },
            Command::Flock,
        )
        .unwrap();
    let route = Route::new(vec![[0.0, 0.0], [0.0, 4.0]], 0.5).unwrap();
    let previous_path = world.add_path(route.clone());
    world.assign_path(group, previous_path).unwrap();
    let preset = ScenePreset::capture(&scene());
    preset.apply(&mut world).unwrap();
    assert_eq!(world.group(group).unwrap().command(), Command::FollowRoute);
    assert_eq!(world.group(group).unwrap().route(), Some(&route));
    assert_eq!(world.paths(), scene().paths());
    assert_ne!(world.paths()[0], route);
}

#[test]
fn save_never_overwrites_existing_files() {
    let path = std::env::temp_dir().join(format!(
        "swarm-preset-{}-{}.scene",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    let preset = ScenePreset::capture(&scene());
    preset.save_new(&path).unwrap();
    assert_eq!(ScenePreset::load(&path).unwrap(), preset);
    assert!(
        ScenePreset::capture(&Simulation::empty(Config::default()).unwrap())
            .save_new(&path)
            .is_err()
    );
    assert_eq!(ScenePreset::load(&path).unwrap(), preset);
    std::fs::remove_file(path).unwrap();
}

#[test]
fn cli_loads_one_environment_for_different_group_parameters() {
    let path = std::env::temp_dir().join(format!(
        "swarm-cli-scene-{}-{}.swarm",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    ScenePreset::capture(&scene()).save_new(&path).unwrap();
    for speed in ["0.5", "2"] {
        let result = std::process::Command::new(env!("CARGO_BIN_EXE_simulation"))
            .arg("--scene")
            .arg(&path)
            .args([
                "--steps", "0", "--group", "0", "0", "2", "1", "7", "boids", "--speed", speed,
            ])
            .output()
            .unwrap();
        assert!(
            result.status.success(),
            "{}",
            String::from_utf8_lossy(&result.stderr)
        );
        let output = String::from_utf8(result.stdout).unwrap();
        assert!(output.contains("\"rectangles\":[{"));
        assert!(output.contains("\"type\":\"paths_replaced\""));
        assert!(output.contains(&format!("\"speed\":{speed}")));
        assert!(output.contains("\"initial_agents\":2"));
    }
    std::fs::remove_file(path).unwrap();
}
