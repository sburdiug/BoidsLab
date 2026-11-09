use rust_playground::{
    playback::{Playback, PlaybackState},
    simulation::{Agent, AgentState, Config, Simulation},
};

fn world() -> Simulation {
    Simulation::new(
        Config::default(),
        vec![Agent::new([0.0, 0.0], [1.0, 0.0]).unwrap()],
    )
    .unwrap()
}

#[test]
fn different_frame_rates_execute_the_same_model_steps() {
    let (mut a, mut b) = (world(), world());
    let (mut slow, mut fast) = (Playback::default(), Playback::default());
    slow.toggle();
    fast.toggle();
    for _ in 0..10 {
        slow.frame(0.1, &mut a).unwrap();
    }
    for _ in 0..60 {
        fast.frame(1.0 / 60.0, &mut b).unwrap();
    }
    assert_eq!(a.steps(), 10);
    assert_eq!(a.agents(), b.agents());
    assert_eq!(a.time(), b.time());
}

#[test]
fn pause_step_stop_and_reset_have_distinct_effects() {
    let mut simulation = world();
    let mut playback = Playback::default();
    playback.frame(10.0, &mut simulation).unwrap();
    assert_eq!(simulation.steps(), 0);
    playback.single_step(&mut simulation).unwrap();
    assert_eq!(simulation.steps(), 1);
    assert_eq!(playback.state(), PlaybackState::Paused);
    playback.toggle();
    playback.frame(0.2, &mut simulation).unwrap();
    assert_eq!(simulation.steps(), 3);
    playback.stop();
    playback.toggle();
    playback.single_step(&mut simulation).unwrap();
    playback.frame(10.0, &mut simulation).unwrap();
    assert_eq!(simulation.steps(), 3);
    playback.reset();
    assert_eq!(playback.state(), PlaybackState::Paused);
    playback.single_step(&mut simulation).unwrap();
    assert_eq!(simulation.steps(), 4);
}

#[test]
fn playback_rate_changes_time_but_not_agent_speed() {
    let mut simulation = world();
    let mut playback = Playback::default();
    playback.set_speed(2.0).unwrap();
    playback.toggle();
    playback.frame(1.0, &mut simulation).unwrap();
    for _ in 0..20 {
        playback.frame(0.0, &mut simulation).unwrap();
    }
    assert_eq!(simulation.steps(), 20);
    assert_eq!(simulation.agents()[0].velocity(), [1.0, 0.0]);
    assert!(playback.set_speed(f64::NAN).is_err());
    assert!(playback.set_speed(0.0).is_err());
}

#[test]
fn catch_up_limits_work_per_frame_without_discarding_steps() {
    let mut simulation = world();
    let mut playback = Playback::default();
    playback.toggle();
    playback.frame(30.0, &mut simulation).unwrap();
    assert!((1..=8).contains(&simulation.steps()));
    assert!((simulation.time() + playback.backlog_seconds() - 30.0).abs() < 1e-10);
    for _ in 0..300 {
        playback.frame(0.0, &mut simulation).unwrap();
    }
    assert_eq!(simulation.steps(), 300);
}

#[test]
fn rendered_motion_is_smooth_between_ticks_at_every_playback_rate() {
    for rate in [0.5, 1.0, 2.0, 4.0] {
        let mut simulation = world();
        let mut playback = Playback::default();
        playback.set_speed(rate).unwrap();
        playback.toggle();
        playback.frame(0.125 / rate, &mut simulation).unwrap();
        let state = simulation.agents().to_vec();
        assert!((playback.interpolated_positions(&simulation)[0][0] - 0.025).abs() < 1e-10);
        assert_eq!(simulation.agents(), state);
        playback.frame(0.025 / rate, &mut simulation).unwrap();
        assert_eq!(simulation.steps(), 1);
        assert!((playback.interpolated_positions(&simulation)[0][0] - 0.05).abs() < 1e-10);
        playback.frame(0.26 / rate, &mut simulation).unwrap();
        for _ in 0..4 {
            playback.frame(0.0, &mut simulation).unwrap();
        }
        assert_eq!(simulation.steps(), 4);
        assert!((playback.interpolated_positions(&simulation)[0][0] - 0.31).abs() < 1e-10);
        playback.toggle();
        assert_eq!(
            playback.interpolated_positions(&simulation)[0],
            simulation.agents()[0].position()
        );
        playback.single_step(&mut simulation).unwrap();
        assert_eq!(
            playback.interpolated_positions(&simulation)[0],
            simulation.agents()[0].position()
        );
    }
}

#[test]
fn collision_markers_remain_at_the_actual_contact_positions() {
    let config = Config {
        separation_weight: 0.0,
        alignment_weight: 0.0,
        cohesion_weight: 0.0,
        ..Config::default()
    };
    let mut simulation = Simulation::new(
        config,
        vec![
            Agent::new([-0.1, 0.0], [1.0, 0.0]).unwrap(),
            Agent::new([0.1, 0.0], [-1.0, 0.0]).unwrap(),
        ],
    )
    .unwrap();
    let mut playback = Playback::default();
    playback.toggle();
    playback.frame(0.125, &mut simulation).unwrap();
    assert!(
        simulation
            .agents()
            .iter()
            .all(|a| a.state() == AgentState::Terminated)
    );
    assert_eq!(
        playback.interpolated_positions(&simulation),
        simulation
            .agents()
            .iter()
            .map(|a| a.position())
            .collect::<Vec<_>>()
    );
}
