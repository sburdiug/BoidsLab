use super::TEXT;
use macroquad::prelude::*;

pub(super) fn arrow(start: Vec2, end: Vec2, color: Color) {
    let delta = end - start;
    draw_line(start.x, start.y, end.x, end.y, 2.0, color);
    if delta.length() > 1.0 {
        let forward = delta.normalize();
        let side = vec2(-forward.y, forward.x);
        draw_triangle(
            end,
            end - forward * 14.0 + side * 6.0,
            end - forward * 14.0 - side * 6.0,
            color,
        );
    }
}

pub(super) fn movement_direction(previous: Vec2, velocity: [f64; 2]) -> Vec2 {
    let speed = velocity[0].hypot(velocity[1]);
    if speed > 0.0 {
        vec2((velocity[0] / speed) as f32, (-velocity[1] / speed) as f32)
    } else {
        previous
    }
}

pub(super) fn copter_outline(center: Vec2, forward: Vec2) -> [Vec2; 5] {
    let side = vec2(-forward.y, forward.x);
    [
        center + forward * 8.0,
        center + forward * 4.0 + side * 5.0,
        center - forward * 5.0 + side * 5.0,
        center - forward * 5.0 - side * 5.0,
        center + forward * 4.0 - side * 5.0,
    ]
}

pub(super) fn draw_copter(center: Vec2, forward: Vec2, color: Color) {
    let p = copter_outline(center, forward);
    for i in 1..4 {
        draw_triangle(p[0], p[i], p[i + 1], color);
    }
    let side = vec2(-forward.y, forward.x);
    draw_triangle(
        p[0],
        center + forward * 4.0 + side * 2.0,
        center + forward * 4.0 - side * 2.0,
        TEXT,
    );
}
