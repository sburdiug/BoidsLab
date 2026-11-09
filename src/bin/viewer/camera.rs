use macroquad::prelude::*;

pub(super) fn screen_point(world: [f64; 2], camera: [f64; 2], zoom: f64, area: Rect) -> Vec2 {
    vec2(
        area.x + area.w / 2.0 + ((world[0] - camera[0]) * zoom) as f32,
        area.y + area.h / 2.0 - ((world[1] - camera[1]) * zoom) as f32,
    )
}

pub(super) fn world_point(screen: Vec2, camera: [f64; 2], zoom: f64, area: Rect) -> [f64; 2] {
    [
        camera[0] + (screen.x - area.x - area.w / 2.0) as f64 / zoom,
        camera[1] - (screen.y - area.y - area.h / 2.0) as f64 / zoom,
    ]
}

pub(super) fn wheel_steps(delta: f32) -> f64 {
    // Miniquad exposes raw WHEEL_DELTA units on Windows, lines elsewhere.
    if cfg!(target_os = "windows") {
        delta as f64 / 120.0
    } else {
        delta as f64
    }
}

pub(super) fn zoom_at(steps: f64) -> f64 {
    80.0 * 1.1_f64.powf(steps)
}

pub(super) fn change_zoom(steps: &mut f64, amount: f64) {
    *steps = (*steps + amount).clamp((10.0_f64 / 80.0).log(1.1), (500.0_f64 / 80.0).log(1.1));
    if steps.abs() < 1e-12 {
        *steps = 0.0;
    }
}
