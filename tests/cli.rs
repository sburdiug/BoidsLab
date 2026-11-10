use std::process::Command;

#[test]
fn default_radius_and_explicit_historical_radius_are_preserved() {
    for (args, expected) in [
        (vec!["--steps", "0"], "\"body_radius\":0.1"),
        (
            vec!["--steps", "0", "--body-radius", "0.025"],
            "\"body_radius\":0.025",
        ),
    ] {
        let output = Command::new(env!("CARGO_BIN_EXE_simulation"))
            .args(args)
            .output()
            .unwrap();
        assert!(output.status.success());
        assert!(String::from_utf8(output.stdout).unwrap().contains(expected));
    }
}

#[test]
fn trace_interval_keeps_initial_final_and_intermediate_events() {
    let path = std::env::temp_dir().join(format!(
        "swarm-interval-{}-{}.jsonl",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    let output = Command::new(env!("CARGO_BIN_EXE_simulation"))
        .args([
            "--agents",
            "1",
            "--steps",
            "5",
            "--trace-every",
            "3",
            "--hold",
            "--trace",
        ])
        .arg(&path)
        .output()
        .unwrap();
    assert!(output.status.success());
    let trace = std::fs::read_to_string(&path).unwrap();
    let lines: Vec<_> = trace.lines().collect();
    assert_eq!(lines.len(), 3);
    assert!(lines[0].contains("\"steps\":0"));
    assert!(lines[1].contains("\"steps\":3"));
    assert!(lines[2].contains("\"steps\":5"));
    assert!(lines[0].contains("\"group_added\""));
    assert!(
        String::from_utf8(output.stdout)
            .unwrap()
            .contains("\"event_scope\":\"full_history\"")
    );
    std::fs::remove_file(path).unwrap();
}

#[test]
fn headless_run_reports_the_completed_horizon() {
    let output = Command::new(env!("CARGO_BIN_EXE_simulation"))
        .args([
            "--agents",
            "1",
            "--steps",
            "10",
            "--seed",
            "42",
            "--direction",
            "1",
            "0",
        ])
        .output()
        .unwrap();
    assert!(output.status.success());
    assert!(output.stderr.is_empty());
    let summary = String::from_utf8(output.stdout).unwrap();
    assert!(summary.contains("\"steps\":10"));
    assert!(summary.contains("\"time\":1"));
    assert!(summary.contains("\"initial_agents\":1"));
    assert!(summary.contains("\"moving\":1"));
}

#[test]
fn invalid_cli_input_fails_without_a_success_summary() {
    for args in [
        vec!["--dt", "0"],
        vec!["--direction", "0", "0"],
        vec!["--gather", "0", "0", "-1"],
        vec!["--agents"],
        vec!["--unknown"],
        vec!["--spread", "2"],
        vec!["--spread", "0.01"],
        vec!["--route-centering", "NaN"],
        vec!["--route-cohesion", "-1"],
        vec!["--direction", "1", "0", "--gather", "0", "0", "1"],
        vec!["--hold", "--spread", "0.8"],
        vec!["--max-acceleration", "0"],
        vec!["--max-braking", "-1"],
        vec!["--max-jerk", "NaN"],
        vec!["--square", "0", "0", "0"],
        vec!["--noise", "0", "0", "2", "-1"],
        vec!["--wind", "0", "0", "2", "NaN", "0"],
        vec!["--waypoint", "0", "0", "--hold"],
        vec!["--route-radius", "1"],
        vec!["--square-angle", "45"],
        vec!["--wind-rect", "0", "0", "2", "4", "90", "-1"],
        vec!["--noise-rect", "0", "0", "0", "4", "30", "0.1"],
        vec!["--waypoint", "0", "0", "--route-width", "0"],
        vec![
            "--waypoint",
            "0",
            "0",
            "--route-width",
            "2",
            "--route-radius",
            "1",
        ],
    ] {
        let output = Command::new(env!("CARGO_BIN_EXE_simulation"))
            .args(args)
            .output()
            .unwrap();
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
        assert!(!output.stderr.is_empty());
    }
}

#[test]
fn rotated_rectangles_are_exported_with_independent_dimensions_and_strength() {
    let output = Command::new(env!("CARGO_BIN_EXE_simulation"))
        .args([
            "--agents",
            "1",
            "--steps",
            "0",
            "--square",
            "5",
            "0",
            "2",
            "--square-angle",
            "45",
            "--wind-rect",
            "0",
            "0",
            "2",
            "8",
            "90",
            "0.7",
            "--noise-rect",
            "0",
            "0",
            "3",
            "7",
            "30",
            "0.04",
            "--hold",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let text = String::from_utf8(output.stdout).unwrap();
    assert!(text.contains("\"angle_degrees\":45"));
    assert!(text.contains("\"width\":2,\"length\":8"));
    assert!(text.contains("\"speed\":0.7"));
    assert!(text.contains("\"width\":3,\"length\":7"));
    assert!(text.contains("\"position_stddev\":0.04"));
}

#[test]
fn corridor_width_and_the_legacy_half_width_flag_agree() {
    let run = |flag: &str, size: &str| {
        Command::new(env!("CARGO_BIN_EXE_simulation"))
            .args([
                "--agents",
                "3",
                "--steps",
                "20",
                "--waypoint",
                "0",
                "0",
                "--waypoint",
                "10",
                "0",
                flag,
                size,
            ])
            .output()
            .unwrap()
    };
    let width = run("--route-width", "2");
    let radius = run("--route-radius", "1");
    assert!(width.status.success() && radius.status.success());
    assert_eq!(width.stdout, radius.stdout);
    assert!(
        String::from_utf8(width.stdout)
            .unwrap()
            .contains("\"width\":2")
    );
}

#[test]
fn scene_trace_is_repeatable_jsonl_and_refuses_to_overwrite_a_file() {
    let nonce = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let path =
        std::env::temp_dir().join(format!("swarm-trace-{}-{nonce}.jsonl", std::process::id()));
    let args = [
        "--agents",
        "3",
        "--steps",
        "3",
        "--square",
        "4",
        "0",
        "1",
        "--wind",
        "0",
        "0",
        "4",
        "0.2",
        "0",
        "--noise",
        "0",
        "0",
        "6",
        "0.02",
        "--waypoint",
        "-3",
        "0",
        "--waypoint",
        "6",
        "0",
        "--route-radius",
        "0.5",
    ];
    let output = Command::new(env!("CARGO_BIN_EXE_simulation"))
        .args(args)
        .arg("--trace")
        .arg(&path)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let text = std::fs::read_to_string(&path).unwrap();
    assert_eq!(text.lines().count(), 4);
    assert!(
        text.lines()
            .all(|line| line.starts_with('{') && line.ends_with('}'))
    );
    assert!(text.contains("\"measured_position\""));
    assert!(text.contains("\"air_velocity\""));
    assert!(text.contains("\"waypoint_index\""));
    let repeated = Command::new(env!("CARGO_BIN_EXE_simulation"))
        .args(args)
        .output()
        .unwrap();
    assert!(repeated.status.success());
    assert_eq!(output.stdout, repeated.stdout);
    let refused = Command::new(env!("CARGO_BIN_EXE_simulation"))
        .args(args)
        .arg("--trace")
        .arg(&path)
        .output()
        .unwrap();
    assert!(!refused.status.success());
    assert_eq!(std::fs::read_to_string(&path).unwrap(), text);
    std::fs::remove_file(path).unwrap();
}

#[test]
fn single_agent_commands_report_arrival() {
    for args in [
        vec!["--hold"],
        vec!["--gather", "2", "3", "1"],
        vec!["--spread", "0.8"],
    ] {
        let output = Command::new(env!("CARGO_BIN_EXE_simulation"))
            .args(["--agents", "1", "--steps", "600"])
            .args(args)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let summary = String::from_utf8(output.stdout).unwrap();
        assert!(summary.contains("\"arrived\":1"));
        assert!(summary.contains("\"terminated\":0"));
        assert!(summary.contains("\"model\":\"boids-2d-v14\""));
    }
}

#[test]
fn spacing_and_route_controls_are_exported() {
    let output = Command::new(env!("CARGO_BIN_EXE_simulation"))
        .args([
            "--steps",
            "0",
            "--neighbor-radius",
            "2",
            "--spread",
            "1.5",
            "--route-centering",
            "0.7",
            "--route-cohesion",
            "0",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let text = String::from_utf8(output.stdout).unwrap();
    assert!(text.contains("\"command\":{\"type\":\"spread\",\"spacing\":1.5}"));
    assert!(text.contains("\"route_centering\":0.7"));
    assert!(text.contains("\"route_cohesion\":0"));
}

#[test]
fn independent_cli_groups_export_their_own_profiles_commands_and_directed_policies() {
    let args = [
        "--steps",
        "5",
        "--group",
        "-3",
        "0",
        "1",
        "0.5",
        "42",
        "boids",
        "--gather",
        "0",
        "3",
        "1",
        "--group",
        "3",
        "0",
        "1",
        "0.5",
        "99",
        "boids",
        "--speed",
        "0.5",
        "--interaction",
        "0",
        "1",
        "avoid",
        "--interaction",
        "1",
        "0",
        "ignore",
        "--rectangle",
        "0",
        "0",
        "4",
        "2",
    ];
    let run = || {
        Command::new(env!("CARGO_BIN_EXE_simulation"))
            .args(args)
            .output()
            .unwrap()
    };
    let a = run();
    let b = run();
    assert!(a.status.success(), "{}", String::from_utf8_lossy(&a.stderr));
    assert_eq!(a.stdout, b.stdout);
    let text = String::from_utf8(a.stdout).unwrap();
    assert!(text.contains("\"initial_agents\":2"));
    assert_eq!(text.matches("\"type\":\"group_added\"").count(), 2);
    assert!(text.contains("\"group_id\":0,\"id\":0"));
    assert!(text.contains("\"group_id\":1,\"id\":1"));
    assert!(text.contains("\"speed\":0.5"));
    assert!(text.contains("\"mode\":\"Avoid\""));
    assert!(text.contains("\"mode\":\"Ignore\""));
    assert!(text.contains("\"width\":4,\"height\":2"));
}
#[test]
fn empty_scene_and_invalid_group_definitions_have_explicit_outcomes() {
    let empty = Command::new(env!("CARGO_BIN_EXE_simulation"))
        .args(["--empty", "--steps", "10"])
        .output()
        .unwrap();
    assert!(empty.status.success());
    let text = String::from_utf8(empty.stdout).unwrap();
    assert!(text.contains("\"groups\":[]"));
    assert!(text.contains("\"agents\":[]"));
    assert!(text.contains("\"time\":1"));
    for args in [
        vec!["--group", "0", "0", "0", "1", "42", "copters"],
        vec!["--group", "0", "0", "1", "1", "42", "unknown"],
        vec!["--interaction", "0", "8", "ignore"],
        vec![
            "--group", "0", "0", "1", "1", "42", "boids", "--group", "5", "0", "1", "1", "42",
            "boids", "--dt", "0.2",
        ],
    ] {
        let out = Command::new(env!("CARGO_BIN_EXE_simulation"))
            .args(args)
            .output()
            .unwrap();
        assert!(!out.status.success());
        assert!(out.stdout.is_empty());
        assert!(!out.stderr.is_empty());
    }
}

#[test]
fn cli_exports_multiple_obstacles() {
    let output = Command::new(env!("CARGO_BIN_EXE_simulation"))
        .args([
            "--empty",
            "--steps",
            "0",
            "--rectangle",
            "0",
            "0",
            "2",
            "3",
            "--rectangle",
            "8",
            "1",
            "4",
            "5",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let result = String::from_utf8(output.stdout).unwrap();
    assert!(result.contains("\"rectangles\":[{\"center\":[0,0],\"width\":2,\"height\":3,\"angle_degrees\":0},{\"center\":[8,1],\"width\":4,\"height\":5,\"angle_degrees\":0}]"));
}

#[test]
fn each_cli_obstacle_has_its_own_rotation() {
    let output = Command::new(env!("CARGO_BIN_EXE_simulation"))
        .args([
            "--empty",
            "--steps",
            "0",
            "--rectangle",
            "0",
            "0",
            "2",
            "3",
            "--obstacle-angle",
            "90",
            "--rectangle",
            "8",
            "0",
            "2",
            "3",
            "--obstacle-angle",
            "0",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let summary = String::from_utf8(output.stdout).unwrap();
    assert!(summary.contains("\"center\":[0,0],\"width\":2,\"height\":3,\"angle_degrees\":90"));
    assert!(summary.contains("\"center\":[8,0],\"width\":2,\"height\":3,\"angle_degrees\":0"));
}

#[test]
fn units_are_present_in_cli_snapshots() {
    let result = Command::new(env!("CARGO_BIN_EXE_simulation"))
        .args(["--empty", "--steps", "0"])
        .output()
        .unwrap();
    assert!(result.status.success());
    let json = String::from_utf8(result.stdout).unwrap();
    assert!(json.contains("\"units\":{\"length\":\"m\",\"time\":\"s\",\"speed\":\"m/s\"}"));
}
