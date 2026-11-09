use super::{
    App, FrameContext, Tool,
    camera::{change_zoom, screen_point, wheel_steps, world_point, zoom_at},
    objects::{pick_object, rotation_handle},
};
use macroquad::{prelude::*, ui::root_ui};
use rust_playground::simulation::Command;

impl App {
    pub(super) fn handle_input(&mut self, ctx: FrameContext) -> bool {
        let FrameContext {
            area,
            mouse,
            on_canvas,
            can_command,
        } = ctx;
        if is_mouse_button_pressed(MouseButton::Left) {
            self.navigation = area.contains(mouse);
        }
        if is_key_pressed(KeyCode::Tab) {
            self.navigation = false;
        }
        if is_key_pressed(KeyCode::Escape) {
            root_ui().clear_input_focus();
            self.selected = None;
            self.selected_object = None;
            self.navigation = true;
            self.draft_route.clear();
            self.finish_tool();
        }
        if is_key_pressed(KeyCode::F11) && !self.world.agents().is_empty() {
            let id = self
                .selected
                .map_or(0, |i| (i + 1) % self.world.agents().len());
            self.select_group(Some(self.world.agents()[id].group_id()));
            self.selected = Some(id);
            self.selected_object = None;
        }
        if is_key_pressed(KeyCode::F9) {
            self.neighbors = !self.neighbors;
        }
        if is_key_pressed(KeyCode::F10) {
            self.camera = [0.0; 2];
            self.zoom = 80.0;
            self.zoom_steps = 0.0;
        }
        if area.contains(mouse) {
            if is_mouse_button_down(MouseButton::Right) || is_mouse_button_down(MouseButton::Middle)
            {
                let d = mouse - self.previous_mouse;
                self.camera[0] -= d.x as f64 / self.zoom;
                self.camera[1] += d.y as f64 / self.zoom;
            }
            let scroll = mouse_wheel().1;
            if scroll != 0.0 {
                change_zoom(&mut self.zoom_steps, wheel_steps(scroll));
            }
        }
        if self.navigation {
            let d = 300.0 * get_frame_time() as f64 / self.zoom;
            if is_key_down(KeyCode::Left) {
                self.camera[0] -= d;
            }
            if is_key_down(KeyCode::Right) {
                self.camera[0] += d;
            }
            if is_key_down(KeyCode::Up) {
                self.camera[1] += d;
            }
            if is_key_down(KeyCode::Down) {
                self.camera[1] -= d;
            }
            if is_key_pressed(KeyCode::Equal) {
                change_zoom(&mut self.zoom_steps, 1.0);
            }
            if is_key_pressed(KeyCode::Minus) {
                change_zoom(&mut self.zoom_steps, -1.0);
            }
            for (key, tool) in [
                (KeyCode::I, Tool::Inspect),
                (KeyCode::A, Tool::AddAgents),
                (KeyCode::V, Tool::Fly),
                (KeyCode::G, Tool::Gather),
                (KeyCode::B, Tool::Spread),
                (KeyCode::L, Tool::Follow),
                (KeyCode::O, Tool::Rectangle),
                (KeyCode::W, Tool::Wind),
                (KeyCode::N, Tool::Noise),
                (KeyCode::P, Tool::Route),
            ] {
                if is_key_pressed(key) {
                    self.choose_tool(tool);
                    if tool == Tool::AddAgents {
                        self.settings_tab = 0;
                    }
                    if tool == Tool::Route {
                        self.settings_tab = 2;
                    }
                }
            }
        }
        if on_canvas
            && self.tool == Tool::Inspect
            && self.pending_object.is_none()
            && is_mouse_button_pressed(MouseButton::Left)
        {
            if let Some((id, object)) = self
                .selected_object
                .and_then(|id| id.get(&self.world).map(|o| (id, o)))
                && rotation_handle(object.area(), self.camera, self.zoom, area).distance(mouse)
                    < 12.0
                && can_command
            {
                self.begin_rotation(id, object);
            } else {
                let selected = if is_key_down(KeyCode::LeftAlt) || is_key_down(KeyCode::RightAlt) {
                    None
                } else {
                    self.world
                        .agents()
                        .iter()
                        .enumerate()
                        .map(|(i, _)| {
                            (
                                i,
                                screen_point(
                                    self.markers[i].position,
                                    self.camera,
                                    self.zoom,
                                    area,
                                )
                                .distance(mouse),
                            )
                        })
                        .filter(|(_, d)| *d <= 15.0)
                        .min_by(|a, b| a.1.total_cmp(&b.1))
                        .map(|(i, _)| i)
                };
                if let Some(i) = selected {
                    let group = self.world.agents()[i].group_id();
                    if self.selected_group != Some(group) {
                        self.select_group(Some(group));
                    }
                }
                self.selected = selected;
                self.selected_object = if selected.is_none() {
                    pick_object(
                        &self.world,
                        world_point(mouse, self.camera, self.zoom, area),
                        self.selected_object,
                    )
                } else {
                    None
                };
                if let Some(o) = self.selected_object.and_then(|id| id.get(&self.world)) {
                    self.object_properties.load(o);
                    self.settings_tab = 3;
                }
            }
        }
        let placement = self.placement_input(ctx);
        let apply_route = can_command
            && self.navigation
            && self.tool == Tool::Route
            && is_key_pressed(KeyCode::Enter);
        if self.navigation && self.tool == Tool::Route && is_key_pressed(KeyCode::Backspace) {
            self.draft_route.pop();
        }
        if can_command && !placement {
            let point = world_point(mouse, self.camera, self.zoom, area);
            if self.tool == Tool::AddAgents
                && on_canvas
                && is_mouse_button_pressed(MouseButton::Left)
            {
                self.add_group_at(point);
            } else if self.tool == Tool::Route
                && on_canvas
                && is_mouse_button_pressed(MouseButton::Left)
            {
                self.draft_route.push(point);
            } else if matches!(
                self.tool,
                Tool::Fly | Tool::Gather | Tool::Spread | Tool::Follow
            ) {
                if on_canvas && is_mouse_button_pressed(MouseButton::Left) {
                    self.gesture = Some(point);
                }
                if self.tool == Tool::Follow {
                    if on_canvas
                        && is_mouse_button_down(MouseButton::Left)
                        && self.gesture.is_some()
                    {
                        self.issue_target(Tool::Follow, point, None);
                    }
                    if is_mouse_button_released(MouseButton::Left) && self.gesture.take().is_some()
                    {
                        self.finish_tool();
                    }
                } else if is_mouse_button_released(MouseButton::Left)
                    && let Some(start) = self.gesture.take()
                    && on_canvas
                {
                    let tool = self.tool;
                    if tool == Tool::Fly {
                        self.flight_guide = Some((start, point));
                        self.issue_target(tool, point, None);
                    } else {
                        let r = (point[0] - start[0]).hypot(point[1] - start[1]);
                        self.issue_target(tool, start, (r * self.zoom > 5.0).then_some(r));
                    }
                }
            }
        }
        self.previous_mouse = mouse;
        let target = zoom_at(self.zoom_steps);
        self.zoom +=
            (target - self.zoom) * (1.0 - (-12.0 * get_frame_time().min(0.1) as f64).exp());
        if (target - self.zoom).abs() < 0.001 {
            self.zoom = target;
        }
        apply_route
    }
    pub(super) fn issue_target(&mut self, tool: Tool, center: [f64; 2], drawn: Option<f64>) {
        let text = if tool == Tool::Spread {
            &self.spread_spacing
        } else {
            &self.target_radius
        };
        let result = drawn
            .map(Ok)
            .unwrap_or_else(|| text.trim().parse::<f64>())
            .map_err(|_| "Invalid radius / gap".to_owned())
            .and_then(|radius| {
                let command = if tool == Tool::Spread {
                    Command::Spread { spacing: radius }
                } else {
                    Command::Gather { center, radius }
                };
                if tool != Tool::Follow
                    || self
                        .selected_group
                        .and_then(|id| self.world.group(id))
                        .is_none_or(|g| g.command() != command)
                {
                    self.command_group(command)?;
                }
                if drawn.is_some() {
                    if tool == Tool::Spread {
                        self.spread_spacing = radius.to_string();
                    } else {
                        self.target_radius = radius.to_string();
                    }
                }
                Ok(())
            });
        match result {
            Ok(()) => {
                self.notice = None;
                if tool != Tool::Fly {
                    self.flight_guide = None;
                }
                if tool != Tool::Follow {
                    self.finish_tool();
                }
            }
            Err(e) => self.notice = Some(format!("Command rejected: {e}")),
        }
    }
}
