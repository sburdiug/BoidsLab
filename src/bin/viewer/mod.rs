use macroquad::{prelude::*, ui::root_ui};
use rust_playground::{
    playback::{Playback, PlaybackState},
    simulation::Simulation,
};
mod camera;
mod canvas;
mod controls;
mod glyphs;
mod groups;
mod input;
mod inspectors;
mod objects;
mod placement;
mod presets;
mod settings;
#[cfg(test)]
mod tests;
mod theme;
use glyphs::movement_direction;
use objects::{ObjectId, ObjectProperties, PendingObject};
use settings::Settings;
pub(crate) use theme::window_conf;
use theme::{label, skin};

const PANEL: f32 = 300.0;
const HEADER: f32 = 74.0;
const FOOTER: f32 = 60.0;
const BG: Color = Color::from_rgba(12, 18, 28, 255);
const SURFACE: Color = Color::from_rgba(21, 30, 43, 255);
const TEXT: Color = Color::from_rgba(236, 243, 252, 255);
const MUTED: Color = Color::from_rgba(165, 182, 202, 255);
const MINT: Color = Color::from_rgba(87, 226, 184, 255);
const ARRIVED: Color = Color::from_rgba(125, 184, 255, 255);
const DEAD: Color = Color::from_rgba(255, 125, 145, 255);
const SELECTED: Color = Color::from_rgba(255, 211, 112, 255);
#[derive(Clone, Copy, PartialEq)]
enum Tool {
    Inspect,
    AddAgents,
    Fly,
    Gather,
    Spread,
    Follow,
    Rectangle,
    Wind,
    Noise,
    Route,
}

#[derive(Clone, Copy)]
struct Marker {
    position: [f64; 2],
    direction: Vec2,
}

pub(crate) struct App {
    selected_group: Option<usize>,
    interaction_target: usize,
    selected_path: Option<usize>,
    rectangle_draft: Option<placement::RectangleDraft>,
    settings: Settings,
    world: Simulation,
    playback: Playback,
    settings_tab: usize,
    object_properties: ObjectProperties,
    preset_path: String,
    preset_status: String,
    selected_object: Option<ObjectId>,
    pending_object: Option<PendingObject>,
    draft_route: Vec<[f64; 2]>,
    markers: Vec<Marker>,
    camera: [f64; 2],
    zoom: f64,
    zoom_steps: f64,
    tool: Tool,
    gesture: Option<[f64; 2]>,
    flight_guide: Option<([f64; 2], [f64; 2])>,
    target_radius: String,
    route_width: String,
    spread_spacing: String,
    notice: Option<String>,
    selected: Option<usize>,
    neighbors: bool,
    error: Option<String>,
    navigation: bool,
    previous_mouse: Vec2,
}

#[derive(Clone, Copy)]
struct FrameContext {
    area: Rect,
    mouse: Vec2,
    on_canvas: bool,
    can_command: bool,
}

impl App {
    pub(crate) fn new() -> Result<Self, String> {
        let settings = Settings::default();
        let world = Simulation::empty(settings.config()?).map_err(|e| e.to_string())?;
        let skin = skin();
        root_ui().push_skin(&skin);
        Ok(Self::from_world(
            settings,
            world,
            Vec2::from(mouse_position()),
        ))
    }
    fn from_world(settings: Settings, world: Simulation, previous_mouse: Vec2) -> Self {
        let markers = vec![
            Marker {
                position: [0.0; 2],
                direction: vec2(0.0, -1.0)
            };
            world.agents().len()
        ];
        Self {
            selected_group: None,
            interaction_target: 0,
            selected_path: None,
            rectangle_draft: None,
            settings,
            world,
            markers,
            playback: Playback::default(),
            settings_tab: 0,
            object_properties: ObjectProperties::default(),
            preset_path: "scene.swarm".into(),
            preset_status: String::new(),
            selected_object: None,
            pending_object: None,
            draft_route: Vec::new(),
            camera: [0.0; 2],
            zoom: 80.0,
            zoom_steps: 0.0,
            tool: Tool::Inspect,
            gesture: None,
            flight_guide: None,
            target_radius: "1".to_owned(),
            route_width: "2".to_owned(),
            spread_spacing: "0.6".to_owned(),
            notice: None,
            selected: None,
            neighbors: true,
            error: None,
            navigation: true,
            previous_mouse,
        }
    }
    pub(crate) fn frame(&mut self) {
        clear_background(BG);
        if screen_width() < 1000.0 || screen_height() < 760.0 {
            if self.playback.state() == PlaybackState::Running {
                self.playback.toggle();
            }
            label(
                "Simulation paused. Resize the window to at least 1000 x 760.",
                25.0,
                55.0,
                22.0,
                TEXT,
            );
            return;
        }
        let area = Rect::new(
            PANEL,
            HEADER,
            screen_width() - PANEL,
            screen_height() - HEADER - FOOTER,
        );
        let mouse = Vec2::from(mouse_position());
        self.sync_markers();
        for ((marker, position), agent) in self
            .markers
            .iter_mut()
            .zip(self.playback.interpolated_positions(&self.world))
            .zip(self.world.agents())
        {
            marker.position = position;
            marker.direction = movement_direction(marker.direction, agent.velocity());
        }
        let card = Rect::new(screen_width() - 268.0, 90.0, 250.0, 294.0);
        let ctx = FrameContext {
            area,
            mouse,
            on_canvas: area.contains(mouse) && !card.contains(mouse),
            can_command: self.playback.state() != PlaybackState::Stopped && self.error.is_none(),
        };
        let apply_route = self.handle_input(ctx);
        self.draw_canvas(ctx);
        let restart = self.draw_playback();
        self.draw_settings(ctx, apply_route);
        self.draw_commands(ctx, restart);
        if let Err(e) = self
            .playback
            .frame(get_frame_time() as f64, &mut self.world)
        {
            self.error = Some(e.to_string());
        }
        self.draw_overlays(ctx);
    }
}
