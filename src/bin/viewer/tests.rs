use super::{
    Tool,
    camera::*,
    glyphs::{agent_outline, movement_direction},
    objects::ObjectProperties,
    settings::Settings,
};
use macroquad::prelude::*;
use rust_playground::simulation::Route;

#[test]
fn scene_tools_validate_properties_and_restart_preserves_the_scene_and_route() {
    let settings = Settings::default();
    let mut world = settings.build().unwrap();
    let mut properties = ObjectProperties::default();
    properties
        .create(Tool::Rectangle, [5.0, 0.0], None)
        .unwrap()
        .apply(&mut world, None)
        .unwrap();
    properties
        .create(Tool::Wind, [0.0, 0.0], None)
        .unwrap()
        .apply(&mut world, None)
        .unwrap();
    properties
        .create(Tool::Noise, [0.0, 0.0], None)
        .unwrap()
        .apply(&mut world, None)
        .unwrap();
    world
        .set_route(Route::new(vec![[0.0, 0.0], [8.0, 0.0]], 0.5).unwrap())
        .unwrap();
    properties.values[10] = "-1".into();
    assert!(properties.create(Tool::Noise, [0.0, 0.0], None).is_err());
    assert_eq!(world.noise_zones().len(), 1);
    let recreated = settings.build_with_scene(Some(&world)).unwrap();
    assert_eq!(recreated.obstacle(), world.obstacle());
    assert_eq!(recreated.wind_zones(), world.wind_zones());
    assert_eq!(recreated.noise_zones(), world.noise_zones());
    assert_eq!(recreated.route(), world.route());
}

#[test]
fn coordinates_round_trip_with_y_up_and_a_panned_camera() {
    let area = Rect::new(300.0, 74.0, 900.0, 704.0);
    let camera = [2.0, -3.0];
    let point = [4.0, 5.0];
    let screen = screen_point(point, camera, 80.0, area);
    let recovered = world_point(screen, camera, 80.0, area);
    assert_eq!(point, recovered);
    assert!(screen.y < area.y + area.h / 2.0);
}

#[test]
fn pending_settings_are_validated_before_replacing_a_run() {
    let mut settings = Settings::default();
    let world = settings.build().unwrap();
    settings.values[4] = "0".into();
    assert!(settings.build().is_err());
    assert_eq!(world.agents().len(), 10);
    assert_eq!(world.steps(), 0);
}

#[test]
fn wheel_zoom_is_reversible_and_keeps_the_center_fixed() {
    let delta = if cfg!(target_os = "windows") {
        120.0
    } else {
        1.0
    };
    let area = Rect::new(300.0, 74.0, 900.0, 704.0);
    let center = vec2(area.x + area.w / 2.0, area.y + area.h / 2.0);
    let camera = [2.0, -3.0];
    let mut steps = 0.0;
    change_zoom(&mut steps, wheel_steps(delta));
    assert!((zoom_at(steps) - 88.0).abs() < 1e-10);
    assert_eq!(world_point(center, camera, zoom_at(steps), area), camera);
    change_zoom(&mut steps, wheel_steps(-delta));
    assert_eq!(zoom_at(steps), 80.0);
    change_zoom(&mut steps, 10000.0);
    assert!((zoom_at(steps) - 500.0).abs() < 1e-10);
}

#[test]
fn movement_marker_follows_velocity_and_remembers_direction_while_stopped() {
    let east = movement_direction(vec2(0.0, -1.0), [1.0, 0.0]);
    assert_eq!(east, vec2(1.0, 0.0));
    assert_eq!(movement_direction(east, [0.0, 0.0]), east);
    let west = movement_direction(east, [-1.0, 0.0]);
    assert_eq!(west, vec2(-1.0, 0.0));
    let outline = agent_outline(vec2(30.0, 20.0), west);
    assert_eq!(outline[0], vec2(22.0, 20.0));
    assert_eq!(outline.len(), 5);
}

#[test]
fn route_weights_are_editable_and_validated_with_other_settings() {
    let mut settings = Settings::default();
    settings.values[14] = "0.7".into();
    settings.values[15] = "0".into();
    let world = settings.build().unwrap();
    assert_eq!(world.config().route_centering, 0.7);
    assert_eq!(world.config().route_cohesion, 0.0);
    settings.values[15] = "-1".into();
    assert!(settings.build().is_err());
}

fn empty_app() -> super::App {
    super::App::from_world(
        Settings::default(),
        rust_playground::simulation::Simulation::empty(Default::default()).unwrap(),
        vec2(0.0, 0.0),
    )
}

#[test]
fn viewer_accepts_more_than_a_thousand_agents() {
    let mut app = empty_app();
    app.settings.values[0] = "1001".into();
    app.settings.values[2] = "60".into();
    app.add_group_at([0.0, 0.0]);
    assert_eq!(app.world.agents().len(), 1001, "{:?}", app.notice);
    app.settings.values[0] = "1".into();
    app.add_group_at([200.0, 0.0]);
    assert_eq!(app.world.agents().len(), 1002, "{:?}", app.notice);
    app.settings.values[0] = "0".into();
    assert!(app.settings.spawn([0.0, 0.0]).is_err());
}
#[test]
fn empty_scene_adds_groups_live_and_finishes_commands_after_assignment() {
    use rust_playground::{playback::PlaybackState, simulation::Command};
    let mut app = empty_app();
    assert!(app.world.agents().is_empty());
    app.settings.values[0] = "2".into();
    app.settings.values[2] = "0.5".into();
    app.choose_tool(Tool::AddAgents);
    app.add_group_at([-3.0, 0.0]);
    assert_eq!(app.world.groups().len(), 1);
    assert!(app.tool == Tool::Inspect);
    assert_eq!(app.markers.len(), 2);
    app.playback.toggle();
    app.add_group_at([3.0, 0.0]);
    assert_eq!(app.playback.state(), PlaybackState::Running);
    assert_eq!(app.world.group(1).unwrap().command(), Command::Flock);
    app.select_group(Some(0));
    app.choose_tool(Tool::Fly);
    app.issue_target(Tool::Fly, [0.0, 4.0], None);
    assert!(app.tool == Tool::Inspect);
    assert!(matches!(
        app.world.group(0).unwrap().command(),
        Command::Gather { .. }
    ));
    assert_eq!(app.world.group(1).unwrap().command(), Command::Flock);
    let hash = app.world.group(0).unwrap().profile_hash();
    app.settings.values[3] = "0.5".into();
    app.apply_profile();
    assert_eq!(app.world.group(0).unwrap().profile_hash(), hash);
    app.playback.toggle();
    app.apply_profile();
    assert_ne!(app.world.group(0).unwrap().profile_hash(), hash);
    app.choose_tool(Tool::Route);
    app.draft_route = vec![[-3.0, 0.0], [-3.0, 5.0]];
    app.save_path();
    assert!(app.tool == Tool::Inspect);
    assert_eq!(app.world.paths().len(), 1);
    assert_eq!(app.world.group(0).unwrap().command(), Command::FollowRoute);
    app.new_scene();
    assert!(app.world.groups().is_empty());
    assert!(app.world.agents().is_empty());
}

#[test]
fn new_groups_default_to_free_boids_even_after_selecting_a_hovering_group() {
    use rust_playground::simulation::Command;
    let mut app = empty_app();
    app.settings.values[0] = "1".into();
    app.add_group_at([-3.0, 0.0]);
    assert_eq!(app.world.group(0).unwrap().command(), Command::Flock);
    app.command_group(Command::Hold).unwrap();
    app.select_group(Some(0));
    app.add_group_at([3.0, 0.0]);
    assert_eq!(app.world.group(1).unwrap().command(), Command::Flock);
}
#[test]
fn rectangle_width_and_height_are_independent_before_rotation() {
    let draft = super::placement::RectangleDraft {
        anchor: [0.0, 0.0],
        width: 4.0,
    };
    let shape = draft.shape([4.0, 2.0]).unwrap();
    assert_eq!(shape.length(), 4.0);
    assert_eq!(shape.width(), 2.0);
    assert_eq!(shape.center(), [2.0, 1.0]);
    let rotated = shape.rotated(std::f64::consts::FRAC_PI_2).unwrap();
    assert_eq!(rotated.length(), 4.0);
    assert_eq!(rotated.width(), 2.0);
}
