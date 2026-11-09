use super::{
    App, FrameContext, MUTED, Marker, SELECTED, TEXT, Tool,
    theme::{button, label},
};
use macroquad::prelude::*;
use rust_playground::{
    playback::PlaybackState,
    simulation::{Command, Interaction, Route},
};

impl App {
    pub(super) fn new_scene(&mut self) {
        match self.settings.config().and_then(|c| {
            rust_playground::simulation::Simulation::empty(c).map_err(|e| e.to_string())
        }) {
            Ok(world) => {
                self.world = world;
                self.playback.reset();
                self.markers.clear();
                self.selected = None;
                self.selected_group = None;
                self.selected_path = None;
                self.selected_object = None;
                self.draft_route.clear();
                self.error = None;
                self.notice = None;
                self.finish_tool();
            }
            Err(e) => self.notice = Some(e),
        }
    }
    pub(super) fn choose_tool(&mut self, tool: Tool) {
        self.tool = tool;
        if tool == Tool::Route {
            self.settings_tab = 2;
        }
        if tool == Tool::AddAgents {
            self.settings_tab = 0;
        }
        self.gesture = None;
        self.pending_object = None;
        self.rectangle_draft = None;
        self.selected_object = None;
        self.draft_route.clear();
    }
    pub(super) fn finish_tool(&mut self) {
        self.draft_route.clear();
        self.tool = Tool::Inspect;
        self.gesture = None;
        self.pending_object = None;
        self.rectangle_draft = None;
    }
    pub(super) fn select_group(&mut self, id: Option<usize>) {
        self.selected_group = id;
        if let Some(g) = id.and_then(|id| self.world.group(id)) {
            self.settings.load(g);
            if let Some(r) = g.route() {
                self.route_width = r.width().to_string();
            }
        }
        self.selected = None;
        self.flight_guide = None;
    }
    pub(super) fn command_group(&mut self, command: Command) -> Result<(), String> {
        let id = self.selected_group.ok_or("Select a group first")?;
        self.world
            .set_group_command(id, command)
            .map_err(|e| e.to_string())
    }
    pub(super) fn can_edit_group(&self) -> bool {
        self.selected_group.is_some() && self.playback.state() == PlaybackState::Paused
    }
    pub(super) fn apply_profile(&mut self) {
        if !self.can_edit_group() {
            self.notice = Some("Pause and select a group to edit its profile".into());
            return;
        }
        let result = (|| {
            let id = self.selected_group.ok_or("Select a group first")?;
            let config = self.settings.config()?;
            self.world
                .set_group_config(id, config)
                .map_err(|e| e.to_string())
        })();
        match result {
            Ok(()) => {
                self.notice = None;
                self.error = None;
            }
            Err(e) => self.notice = Some(e),
        }
    }
    pub(super) fn add_group_at(&mut self, center: [f64; 2]) {
        let result = (|| {
            let config = self.settings.config()?;
            let spawn = self.settings.spawn(center)?;
            if self.world.groups().is_empty() && self.world.steps() == 0 {
                self.world
                    .set_time_step_before_start(config.dt)
                    .map_err(|e| e.to_string())?;
            }
            self.world
                .add_group(config, spawn, Command::Flock)
                .map_err(|e| e.to_string())
        })();
        match result {
            Ok(id) => {
                self.select_group(Some(id));
                self.sync_markers();
                self.notice = None;
                self.finish_tool();
            }
            Err(e) => self.notice = Some(e),
        }
    }
    pub(super) fn sync_markers(&mut self) {
        for a in &self.world.agents()[self.markers.len().min(self.world.agents().len())..] {
            self.markers.push(Marker {
                position: a.position(),
                direction: super::glyphs::movement_direction(vec2(0.0, -1.0), a.velocity()),
            });
        }
        self.markers.truncate(self.world.agents().len());
    }
    pub(super) fn save_path(&mut self) {
        let result = self
            .route_width
            .trim()
            .parse::<f64>()
            .map_err(|_| "Invalid corridor width".to_owned())
            .and_then(|w| Route::new(self.draft_route.clone(), w * 0.5).map_err(|e| e.to_string()));
        match result {
            Ok(route) => {
                let path = self.world.add_path(route);
                self.selected_path = Some(path);
                self.draft_route.clear();
                self.notice = None;
                if let Some(g) = self.selected_group
                    && let Err(e) = self.world.assign_path(g, path)
                {
                    self.notice = Some(format!("Path saved; assignment failed: {e}"));
                }
                self.flight_guide = None;
                self.finish_tool();
            }
            Err(e) => self.notice = Some(e),
        }
    }
    pub(super) fn draw_groups(&mut self, ctx: FrameContext) {
        let n = self.world.groups().len();
        let tint = self
            .selected_group
            .and_then(|id| self.world.group(id))
            .map_or(TEXT, |g| {
                let [r, g, b, a] = g.color();
                Color::from_rgba(r, g, b, a)
            });
        label(
            &self
                .selected_group
                .map_or("No group selected".into(), |id| {
                    format!(
                        "G{id} / {} agents / {:08x}",
                        self.world.group(id).unwrap().agent_range().len(),
                        self.world.group(id).unwrap().profile_hash() as u32
                    )
                }),
            16.0,
            300.0,
            16.0,
            tint,
        );
        if self.selected_group.is_some() {
            draw_rectangle(272.0, 288.0, 12.0, 12.0, tint);
        }
        if button("Prev", 16.0, 309.0, 80.0, n > 0, false) {
            let id = self
                .selected_group
                .map_or(n - 1, |id| if id == 0 { n - 1 } else { id - 1 });
            self.select_group(Some(id));
        }
        if button("Next", 110.0, 309.0, 80.0, n > 0, false) {
            let id = self.selected_group.map_or(0, |id| (id + 1) % n);
            self.select_group(Some(id));
        }
        if button("None", 204.0, 309.0, 80.0, true, false) {
            self.select_group(None);
        }
        for (slot, i) in [0, 1, 2].into_iter().enumerate() {
            let x = 16.0 + (slot % 2) as f32 * 140.0;
            let y = 365.0 + (slot / 2) as f32 * 44.0;
            label(
                ["New count", "New seed", "Spawn radius (m)"][slot],
                x,
                y,
                15.0,
                MUTED,
            );
            macroquad::ui::widgets::Editbox::new(100 + i as u64, vec2(128.0, 26.0))
                .position(vec2(x, y + 5.0))
                .multiline(false)
                .ui(&mut macroquad::ui::root_ui(), &mut self.settings.values[i]);
        }
        if button(
            "Add [A]",
            16.0,
            454.0,
            128.0,
            ctx.can_command,
            self.tool == Tool::AddAgents,
        ) {
            self.choose_tool(Tool::AddAgents);
        }
        if button(
            "Apply profile",
            156.0,
            454.0,
            128.0,
            self.can_edit_group(),
            false,
        ) {
            self.apply_profile();
        }
        if n > 1
            && let Some(source) = self.selected_group
        {
            self.interaction_target %= n;
            if self.interaction_target == source {
                self.interaction_target = (source + 1) % n;
            }
            if macroquad::ui::widgets::Button::new(
                format!("To group {} >", self.interaction_target).as_str(),
            )
            .position(vec2(16.0, 490.0))
            .size(vec2(268.0, 20.0))
            .ui(&mut macroquad::ui::root_ui())
            {
                self.interaction_target = (self.interaction_target + 1) % n;
                if self.interaction_target == source {
                    self.interaction_target = (source + 1) % n;
                }
            }
            let current = self
                .world
                .interaction(source, self.interaction_target)
                .unwrap();
            for (i, (name, mode)) in [
                ("React", Interaction::React),
                ("Avoid", Interaction::Avoid),
                ("Ignore", Interaction::Ignore),
            ]
            .into_iter()
            .enumerate()
            {
                if button(
                    name,
                    16.0 + i as f32 * 94.0,
                    512.0,
                    80.0,
                    self.can_edit_group(),
                    current == mode,
                ) {
                    match self
                        .world
                        .set_interaction(source, self.interaction_target, mode)
                    {
                        Ok(()) => self.notice = None,
                        Err(e) => self.notice = Some(e.to_string()),
                    }
                }
            }
        } else if let Some(g) = self.selected_group.and_then(|id| self.world.group(id)) {
            label(
                &format!("Profile {:016x}", g.profile_hash()),
                16.0,
                522.0,
                14.0,
                SELECTED,
            );
        }
    }
    pub(super) fn draw_path_assignment(&mut self, ctx: FrameContext) {
        let n = self.world.paths().len();
        if button(
            &format!(
                "Path {} / {n}",
                self.selected_path.map_or("-".into(), |id| id.to_string())
            ),
            16.0,
            454.0,
            128.0,
            n > 0,
            false,
        ) {
            self.selected_path = Some(self.selected_path.map_or(0, |id| (id + 1) % n));
        }
        if button(
            "Save / assign",
            16.0,
            494.0,
            128.0,
            ctx.can_command && !self.draft_route.is_empty(),
            false,
        ) {
            self.save_path();
        }
        if button(
            "Apply profile",
            156.0,
            494.0,
            128.0,
            self.can_edit_group(),
            false,
        ) {
            self.apply_profile();
        }
        if button(
            "Assign path",
            156.0,
            454.0,
            128.0,
            ctx.can_command && self.selected_group.is_some() && self.selected_path.is_some(),
            false,
        ) {
            match self
                .world
                .assign_path(self.selected_group.unwrap(), self.selected_path.unwrap())
            {
                Ok(()) => {
                    self.notice = None;
                    self.finish_tool();
                }
                Err(e) => self.notice = Some(e.to_string()),
            }
        }
    }
}
