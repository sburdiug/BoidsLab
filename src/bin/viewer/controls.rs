use super::{
    App, FrameContext, HEADER, MUTED, Marker, PANEL, SELECTED, SURFACE, TEXT, Tool,
    theme::{button, label},
};
use macroquad::{
    prelude::*,
    ui::{root_ui, widgets},
};
use rust_playground::{
    playback::PlaybackState,
    simulation::{AgentState, Command},
};

impl App {
    pub(super) fn draw_playback(&mut self) -> bool {
        draw_rectangle(0.0, 0.0, PANEL, screen_height(), SURFACE);
        draw_rectangle(0.0, 0.0, screen_width(), HEADER, SURFACE);
        label("SWARM SIMULATOR", 16.0, 32.0, 25.0, TEXT);
        label("2D / fixed-step core", 16.0, 55.0, 17.0, MUTED);
        if button(
            "New scene",
            screen_width() - 284.0,
            14.0,
            128.0,
            self.playback.state() != PlaybackState::Running,
            false,
        ) {
            self.new_scene();
        }
        if button(
            "Reset view",
            screen_width() - 144.0,
            14.0,
            128.0,
            true,
            false,
        ) {
            self.camera = [0.0; 2];
            self.zoom_steps = 0.0;
            self.zoom = 80.0;
        }
        label(
            &format!(
                "{:?}  /  time {:.2}s  /  step {}",
                self.playback.state(),
                self.world.time(),
                self.world.steps()
            ),
            PANEL + 20.0,
            31.0,
            23.0,
            TEXT,
        );
        let counts = |state| {
            self.world
                .agents()
                .iter()
                .filter(|a| a.state() == state)
                .count()
        };
        label(
            &format!(
                "Moving {}    Hovering {}    Terminated {}    Zoom {:.0}%",
                counts(AgentState::Moving),
                counts(AgentState::Arrived),
                counts(AgentState::Terminated),
                self.zoom / 80.0 * 100.0
            ),
            PANEL + 20.0,
            57.0,
            18.0,
            MUTED,
        );
        label("PLAYBACK", 16.0, 96.0, 17.0, MUTED);
        if self.playback.backlog_seconds() > 2.0 * self.world.config().dt {
            label(
                &format!("Lag {:.1}s", self.playback.backlog_seconds()),
                156.0,
                96.0,
                16.0,
                SELECTED,
            );
        }
        let enabled = self.playback.state() != PlaybackState::Stopped && self.error.is_none();
        let run_label = if self.playback.state() == PlaybackState::Running {
            "Pause [F5]"
        } else {
            "Run [F5]"
        };
        if button(run_label, 16.0, 110.0, 128.0, enabled, false)
            || (enabled && is_key_pressed(KeyCode::F5))
        {
            self.playback.toggle();
        }
        if (button("Step [F6]", 156.0, 110.0, 128.0, enabled, false)
            || (enabled && is_key_pressed(KeyCode::F6)))
            && let Err(e) = self.playback.single_step(&mut self.world)
        {
            self.error = Some(e.to_string());
        }
        let restart =
            button("Restart [F8]", 16.0, 154.0, 128.0, true, false) || is_key_pressed(KeyCode::F8);
        if button("Stop [F7]", 156.0, 154.0, 128.0, true, false) || is_key_pressed(KeyCode::F7) {
            self.playback.stop();
        }
        for (i, rate) in [0.5, 1.0, 2.0, 4.0].into_iter().enumerate() {
            if button(
                &format!("{rate}x"),
                16.0 + i as f32 * 70.0,
                200.0,
                58.0,
                true,
                self.playback.speed() == rate,
            ) && let Err(e) = self.playback.set_speed(rate)
            {
                self.error = Some(e.to_string());
            }
        }
        if is_key_pressed(KeyCode::F3) {
            let _ = self
                .playback
                .set_speed((self.playback.speed() / 2.0).max(0.5));
        }
        if is_key_pressed(KeyCode::F4) {
            let _ = self
                .playback
                .set_speed((self.playback.speed() * 2.0).min(4.0));
        }
        restart
    }
}
impl App {
    pub(super) fn draw_commands(&mut self, ctx: FrameContext, restart: bool) {
        let FrameContext { can_command, .. } = ctx;
        let can_swarm = can_command && self.selected_group.is_some();
        let command = self
            .selected_group
            .and_then(|id| self.world.group(id))
            .map(|g| g.command());
        label("LIVE SWARM CONTROL", 16.0, 562.0, 17.0, MUTED);
        for (i, (name, choice)) in [
            ("Fly [V]", Tool::Fly),
            ("Gather [G]", Tool::Gather),
            ("Spread [B]", Tool::Spread),
            ("Inspect [I]", Tool::Inspect),
            ("Follow [L]", Tool::Follow),
        ]
        .into_iter()
        .enumerate()
        {
            if button(
                name,
                16.0 + (i % 3) as f32 * 94.0,
                576.0 + (i / 3) as f32 * 44.0,
                80.0,
                choice == Tool::Inspect || can_swarm,
                self.tool == choice,
            ) {
                self.choose_tool(choice);
            }
        }
        if button(
            "Hover",
            204.0,
            620.0,
            80.0,
            can_swarm,
            command == Some(Command::Hold),
        ) {
            match self.command_group(Command::Hold) {
                Ok(()) => {
                    self.finish_tool();
                    self.flight_guide = None;
                    self.notice = None;
                }
                Err(e) => self.notice = Some(e.to_string()),
            }
        }
        label(
            if self.tool == Tool::Route {
                "Route width"
            } else if self.tool == Tool::Spread {
                "Neighbor gap"
            } else {
                "Target radius"
            },
            16.0,
            683.0,
            17.0,
            MUTED,
        );
        widgets::Editbox::new(200, vec2(128.0, 26.0))
            .position(vec2(16.0, 691.0))
            .multiline(false)
            .ui(
                &mut root_ui(),
                if self.tool == Tool::Route {
                    &mut self.route_width
                } else if self.tool == Tool::Spread {
                    &mut self.spread_spacing
                } else {
                    &mut self.target_radius
                },
            );
        if button(
            "Free Boids",
            156.0,
            685.0,
            128.0,
            can_swarm,
            command == Some(Command::Flock),
        ) {
            match self.command_group(Command::Flock) {
                Ok(()) => {
                    self.finish_tool();
                    self.flight_guide = None;
                    self.notice = None;
                }
                Err(e) => self.notice = Some(e.to_string()),
            }
        }
        if restart {
            match self.world.restarted() {
                Ok(new_world) => {
                    self.world = new_world;
                    self.markers = vec![
                        Marker {
                            position: [0.0; 2],
                            direction: vec2(0.0, -1.0)
                        };
                        self.world.agents().len()
                    ];
                    self.playback.reset();
                    self.selected = None;
                    self.error = None;
                    self.camera = [0.0; 2];
                    self.zoom = 80.0;
                    self.zoom_steps = 0.0;
                    self.gesture = None;
                    self.finish_tool();
                    self.flight_guide = None;
                    self.notice = None;
                    self.pending_object = None;
                    self.selected_object = None;
                    self.draft_route.clear();
                }
                Err(message) => {
                    if self.playback.state() == PlaybackState::Running {
                        self.playback.toggle();
                    }
                    self.notice =
                        Some(format!("Restart rejected: {message}. Previous scene kept."));
                }
            }
        }
    }
}
