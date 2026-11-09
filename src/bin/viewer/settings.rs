use super::{
    App, FrameContext, MUTED, Tool,
    objects::{OBJECT_LABELS, ObjectProperties},
    theme::{button, label},
};
use macroquad::{
    prelude::*,
    ui::{root_ui, widgets},
};
use rust_playground::simulation::{Config, Group, SpawnConfig};
use std::str::FromStr;

const LABELS: [&str; 17] = [
    "Agents",
    "Seed",
    "Spawn radius (m)",
    "Speed (m/s)",
    "Time step (s)",
    "Boids response",
    "Separation weight",
    "Alignment weight",
    "Cohesion weight",
    "View radius (m)",
    "Separation (m)",
    "Body radius (m)",
    "Obstacle gap (m)",
    "Obstacle weight",
    "Route centering",
    "Route cohesion",
    "Goal weight",
];

pub(super) struct Settings {
    pub(super) values: [String; 17],
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            values: [
                "10", "42", "3", "1", "0.1", "0.1", "10", "1", "3", "1", "0.3", "0.025", "0.15",
                "1", "0.25", "1", "15",
            ]
            .map(str::to_owned),
        }
    }
}

impl Settings {
    fn number<T: FromStr>(&self, i: usize) -> Result<T, String> {
        self.values[i]
            .trim()
            .parse()
            .map_err(|_| format!("Invalid {}", LABELS[i]))
    }
    pub(super) fn config(&self) -> Result<Config, String> {
        let value = |i| self.number::<f64>(i);
        Ok(Config {
            speed: value(3)?,
            dt: value(4)?,
            turn_response: value(5)?,
            separation_weight: value(6)?,
            alignment_weight: value(7)?,
            cohesion_weight: value(8)?,
            neighbor_radius: value(9)?,
            separation_radius: value(10)?,
            body_radius: value(11)?,
            obstacle_clearance: value(12)?,
            obstacle_weight: value(13)?,
            route_centering: value(14)?,
            route_cohesion: value(15)?,
            command_weight: value(16)?,
        })
    }
    pub(super) fn spawn(&self, center: [f64; 2]) -> Result<SpawnConfig, String> {
        let count = self.number(0)?;
        if count == 0 {
            return Err("Group count must be positive".into());
        }
        Ok(SpawnConfig {
            count,
            center,
            seed: self.number(1)?,
            radius: self.number(2)?,
        })
    }
    pub(super) fn load(&mut self, group: &Group) {
        let c = group.config();
        let s = group.spawn();
        self.values = [
            s.count.to_string(),
            s.seed.to_string(),
            s.radius.to_string(),
            c.speed.to_string(),
            c.dt.to_string(),
            c.turn_response.to_string(),
            c.separation_weight.to_string(),
            c.alignment_weight.to_string(),
            c.cohesion_weight.to_string(),
            c.neighbor_radius.to_string(),
            c.separation_radius.to_string(),
            c.body_radius.to_string(),
            c.obstacle_clearance.to_string(),
            c.obstacle_weight.to_string(),
            c.route_centering.to_string(),
            c.route_cohesion.to_string(),
            c.command_weight.to_string(),
        ];
    }
    #[cfg(test)]
    pub(super) fn build(&self) -> Result<rust_playground::simulation::Simulation, String> {
        self.build_with_scene(None)
    }
    #[cfg(test)]
    pub(super) fn build_with_scene(
        &self,
        previous: Option<&rust_playground::simulation::Simulation>,
    ) -> Result<rust_playground::simulation::Simulation, String> {
        use rust_playground::simulation::{Command, Simulation};
        let s = self.spawn([0.0; 2])?;
        let mut world = Simulation::seeded_with_obstacle(
            self.config()?,
            s.count,
            s.radius,
            s.seed,
            previous.and_then(|p| p.obstacle()),
        )
        .map_err(|e| e.to_string())?;
        if let Some(p) = previous {
            for z in p.wind_zones() {
                world.add_wind(*z);
            }
            for z in p.noise_zones() {
                world.add_noise(*z);
            }
        }
        world
            .set_command(Command::Hold)
            .map_err(|e| e.to_string())?;
        if let Some(r) = previous.and_then(|p| p.route()) {
            world.set_route(r.clone()).map_err(|e| e.to_string())?;
        }
        Ok(world)
    }
}

impl App {
    pub(super) fn draw_settings(&mut self, ctx: FrameContext, mut apply_route: bool) {
        let FrameContext { can_command, .. } = ctx;
        for (i, name) in ["Group", "Flight", "Swarm", "Scene", "Preset"]
            .into_iter()
            .enumerate()
        {
            if button(
                name,
                16.0 + i as f32 * 56.0,
                246.0,
                52.0,
                true,
                self.settings_tab == i,
            ) {
                self.settings_tab = i;
            }
        }
        let fields: &[usize] = match self.settings_tab {
            0 => &[],
            1 => &[3, 4, 9, 10, 11, 12, 13, 16],
            2 => &[5, 6, 7, 8, 14, 15],
            _ => &[],
        };
        for (slot, &i) in fields.iter().enumerate() {
            let text = &mut self.settings.values[i];
            let x = 16.0 + (slot % 2) as f32 * 140.0;
            let y = 302.0 + (slot / 2) as f32 * if self.settings_tab == 1 { 40.0 } else { 44.0 };
            label(LABELS[i], x, y, 16.0, MUTED);
            widgets::Editbox::new(100 + i as u64, vec2(128.0, 26.0))
                .position(vec2(x, y + 6.0))
                .multiline(false)
                .ui(&mut root_ui(), text);
        }
        if self.settings_tab == 0 {
            self.draw_groups(ctx);
        }
        if self.settings_tab == 3 {
            for (i, (name, choice)) in [
                ("Rectangle [O]", Tool::Rectangle),
                ("Wind [W]", Tool::Wind),
                ("Noise [N]", Tool::Noise),
                ("Route [P]", Tool::Route),
            ]
            .into_iter()
            .enumerate()
            {
                if button(
                    name,
                    16.0 + (i % 2) as f32 * 140.0,
                    342.0 + (i / 2) as f32 * 38.0,
                    128.0,
                    can_command,
                    self.tool == choice,
                ) {
                    self.choose_tool(choice);
                }
            }
            let context = self
                .pending_object
                .as_ref()
                .map(|p| p.object.tool())
                .or_else(|| {
                    self.selected_object
                        .and_then(|id| id.get(&self.world))
                        .map(|o| o.tool())
                })
                .unwrap_or(self.tool);
            for (slot, &i) in ObjectProperties::fields(context).iter().enumerate() {
                let x = 16.0 + (slot % 2) as f32 * 140.0;
                let y = 433.0 + (slot / 2) as f32 * 40.0;
                label(OBJECT_LABELS[i], x, y, 14.0, MUTED);
                widgets::Editbox::new(300 + i as u64, vec2(128.0, 22.0))
                    .position(vec2(x, y + 4.0))
                    .multiline(false)
                    .ui(&mut root_ui(), &mut self.object_properties.values[i]);
            }
            if let Some(pending) = &mut self.pending_object
                && let Ok(object) =
                    self.object_properties
                        .create(context, pending.object.area().center(), None)
            {
                pending.object = object;
            }
            let button_name = if context == Tool::Route {
                "Save / assign"
            } else {
                "Apply object"
            };
            let has_target = self.pending_object.is_some()
                || self
                    .selected_object
                    .and_then(|id| id.get(&self.world))
                    .is_some();
            if button(
                button_name,
                16.0,
                506.0,
                128.0,
                can_command
                    && if context == Tool::Route {
                        !self.draft_route.is_empty()
                    } else {
                        has_target
                    },
                false,
            ) {
                if context == Tool::Route {
                    apply_route = true;
                } else {
                    let target = self
                        .pending_object
                        .as_ref()
                        .map(|p| (p.id, p.object))
                        .or_else(|| {
                            self.selected_object
                                .and_then(|id| id.get(&self.world).map(|o| (Some(id), o)))
                        });
                    if let Some((id, object)) = target {
                        let updated = self
                            .object_properties
                            .create(context, object.area().center(), None)
                            .and_then(|object| object.apply(&mut self.world, id));
                        match updated {
                            Ok(id) => {
                                self.selected_object = Some(id);
                                self.pending_object = None;
                                self.tool = Tool::Inspect;
                                self.notice = None;
                            }
                            Err(e) => self.notice = Some(e),
                        }
                    }
                }
            }
            if button("Clear objects", 156.0, 506.0, 128.0, can_command, false) {
                match self.world.set_obstacle(None) {
                    Ok(()) => {
                        self.world.clear_fields();
                        self.notice = None;
                        self.selected_object = None;
                        self.pending_object = None;
                    }
                    Err(e) => self.notice = Some(e.to_string()),
                }
            }
        }
        if self.settings_tab == 2 {
            self.draw_path_assignment(ctx);
        }
        if self.settings_tab == 4 {
            self.draw_presets();
        }
        if self.settings_tab == 1
            && button(
                "Apply profile",
                16.0,
                506.0,
                268.0,
                self.can_edit_group(),
                false,
            )
        {
            self.apply_profile();
        }
        if apply_route && !self.draft_route.is_empty() {
            self.save_path();
        }
    }
}
