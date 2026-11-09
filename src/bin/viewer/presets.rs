use super::{
    App, MUTED, TEXT,
    theme::{button, label},
};
use macroquad::{
    prelude::*,
    ui::{root_ui, widgets},
};
use rust_playground::{playback::PlaybackState, simulation::ScenePreset};

impl App {
    pub(super) fn save_preset(&mut self) {
        self.preset_status =
            match ScenePreset::capture(&self.world).save_new(self.preset_path.trim()) {
                Ok(()) => "Saved. Groups were not included.".into(),
                Err(e) => format!("Save failed: {e}"),
            };
    }

    pub(super) fn load_preset(&mut self) {
        if self.playback.state() == PlaybackState::Running {
            self.preset_status = "Pause before loading a preset.".into();
            return;
        }
        let result = ScenePreset::load(self.preset_path.trim())
            .and_then(|preset| preset.apply(&mut self.world));
        self.preset_status = match result {
            Ok(()) => {
                self.selected_path = None;
                self.selected_object = None;
                self.finish_tool();
                "Loaded. Groups and commands kept.".into()
            }
            Err(e) => format!("Load failed: {e}"),
        };
    }

    pub(super) fn draw_presets(&mut self) {
        label("Scene preset file", 16.0, 305.0, 16.0, MUTED);
        widgets::Editbox::new(400, vec2(268.0, 26.0))
            .position(vec2(16.0, 311.0))
            .multiline(false)
            .ui(&mut root_ui(), &mut self.preset_path);
        if button("Save new", 16.0, 353.0, 128.0, true, false) {
            self.save_preset();
        }
        if button(
            "Load",
            156.0,
            353.0,
            128.0,
            self.playback.state() != PlaybackState::Running,
            false,
        ) {
            self.load_preset();
        }
        label("Environment + saved paths", 16.0, 416.0, 15.0, MUTED);
        label("Groups and commands stay", 16.0, 438.0, 15.0, MUTED);
        for (i, line) in self.preset_status.as_bytes().chunks(30).take(4).enumerate() {
            label(
                &String::from_utf8_lossy(line),
                16.0,
                468.0 + i as f32 * 19.0,
                15.0,
                TEXT,
            );
        }
    }
}
