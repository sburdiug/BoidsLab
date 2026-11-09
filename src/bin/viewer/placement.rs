use super::{
    App, FrameContext, Tool,
    camera::world_point,
    objects::{ObjectId, PendingObject, SceneObject},
};
use macroquad::prelude::*;
use rust_playground::simulation::Rectangle;

pub(super) struct RectangleDraft {
    pub anchor: [f64; 2],
    pub width: f64,
}
impl RectangleDraft {
    pub fn shape(&self, point: [f64; 2]) -> Result<Rectangle, String> {
        let height = point[1] - self.anchor[1];
        Rectangle::new(
            [
                self.anchor[0] + self.width * 0.5,
                self.anchor[1] + height * 0.5,
            ],
            height.abs(),
            self.width.abs(),
        )
        .map_err(|e| e.to_string())
    }
}
impl App {
    pub(super) fn placement_input(&mut self, ctx: FrameContext) -> bool {
        if !ctx.can_command {
            return false;
        }
        let point = world_point(ctx.mouse, self.camera, self.zoom, ctx.area);
        if self.pending_object.is_some() {
            let pending = self.pending_object.as_mut().unwrap();
            if ctx.on_canvas {
                let center = pending.object.area().center();
                if ctx.mouse.distance(self.previous_mouse) > 0.1
                    && (point[0] - center[0]).hypot(point[1] - center[1]) * self.zoom > 5.0
                {
                    match pending
                        .object
                        .rotated((point[1] - center[1]).atan2(point[0] - center[0]))
                    {
                        Ok(o) => {
                            pending.object = o;
                            self.object_properties.load(o);
                        }
                        Err(e) => self.notice = Some(e),
                    }
                }
            }
            let commit = if pending.dragging {
                is_mouse_button_released(MouseButton::Left)
            } else {
                ctx.on_canvas && is_mouse_button_pressed(MouseButton::Left)
            };
            if commit {
                let pending = self.pending_object.as_ref().unwrap();
                match pending.object.apply(&mut self.world, pending.id) {
                    Ok(id) => {
                        self.selected_object = Some(id);
                        self.selected = None;
                        self.notice = None;
                        self.finish_tool();
                        self.settings_tab = 3;
                    }
                    Err(e) => self.notice = Some(e),
                }
            }
            return true;
        }
        if self.tool == Tool::Rectangle {
            if let Some(draft) = &self.rectangle_draft {
                if ctx.on_canvas && is_mouse_button_pressed(MouseButton::Left) {
                    match draft.shape(point) {
                        Ok(shape) if shape.width() * self.zoom > 5.0 => {
                            let object = SceneObject::Obstacle(shape);
                            self.object_properties.load(object);
                            self.pending_object = Some(PendingObject {
                                object,
                                id: None,
                                dragging: false,
                            });
                            self.rectangle_draft = None;
                            self.notice = None;
                        }
                        _ => self.notice = Some("Choose a nonzero height, then click".into()),
                    }
                }
            } else if ctx.on_canvas && is_mouse_button_pressed(MouseButton::Left) {
                self.gesture = Some(point);
            } else if is_mouse_button_released(MouseButton::Left)
                && let Some(anchor) = self.gesture.take()
            {
                if ctx.on_canvas && (point[0] - anchor[0]).abs() * self.zoom > 5.0 {
                    self.rectangle_draft = Some(RectangleDraft {
                        anchor,
                        width: point[0] - anchor[0],
                    });
                    self.notice = None;
                } else {
                    self.notice = Some("Drag horizontally to choose the width".into());
                }
            }
            return true;
        }
        if matches!(self.tool, Tool::Wind | Tool::Noise) {
            if ctx.on_canvas && is_mouse_button_pressed(MouseButton::Left) {
                self.gesture = Some(point);
            }
            if is_mouse_button_released(MouseButton::Left)
                && let Some(start) = self.gesture.take()
                && ctx.on_canvas
            {
                let moved = (point[0] - start[0]).hypot(point[1] - start[1]) * self.zoom > 5.0;
                let object = if moved {
                    self.object_properties
                        .drawn_dimensions(self.tool, start, point)
                        .and_then(|d| self.object_properties.create(self.tool, start, Some(d)))
                } else {
                    self.object_properties.create(self.tool, start, None)
                };
                match object {
                    Ok(object) => {
                        self.object_properties.load(object);
                        self.pending_object = Some(PendingObject {
                            object,
                            id: None,
                            dragging: false,
                        });
                        self.notice = None;
                    }
                    Err(e) => self.notice = Some(e),
                }
            }
            return true;
        }
        false
    }
    pub(super) fn begin_rotation(&mut self, id: ObjectId, object: SceneObject) {
        self.pending_object = Some(PendingObject {
            id: Some(id),
            object,
            dragging: true,
        });
    }
}
