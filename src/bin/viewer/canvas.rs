use super::{
    ARRIVED, App, BG, DEAD, FrameContext, MUTED, Marker, SELECTED, Tool,
    camera::{screen_point, world_point},
    glyphs::{arrow, draw_agent},
    objects::object_outline,
    theme::label,
};
use macroquad::prelude::*;
use rust_playground::simulation::{AgentState, Command, Rectangle, Simulation};

pub(super) fn grid(camera: [f64; 2], zoom: f64, area: Rect) {
    let spacing = 10_f64.powf((70.0 / zoom).log10().ceil());
    let lo = world_point(vec2(area.x, area.y + area.h), camera, zoom, area);
    let hi = world_point(vec2(area.x + area.w, area.y), camera, zoom, area);
    let color = Color::from_rgba(28, 41, 57, 255);
    for i in (lo[0] / spacing).floor() as i64..=(hi[0] / spacing).ceil() as i64 {
        let p = screen_point([i as f64 * spacing, 0.0], camera, zoom, area);
        draw_line(p.x, area.y, p.x, area.y + area.h, 1.0, color);
    }
    for i in (lo[1] / spacing).floor() as i64..=(hi[1] / spacing).ceil() as i64 {
        let p = screen_point([0.0, i as f64 * spacing], camera, zoom, area);
        draw_line(area.x, p.y, area.x + area.w, p.y, 1.0, color);
    }
    let origin = screen_point([0.0, 0.0], camera, zoom, area);
    draw_line(
        area.x,
        origin.y,
        area.x + area.w,
        origin.y,
        1.5,
        Color::from_rgba(65, 88, 113, 255),
    );
    draw_line(
        origin.x,
        area.y,
        origin.x,
        area.y + area.h,
        1.5,
        Color::from_rgba(65, 88, 113, 255),
    );
    label("(0, 0) m", origin.x + 7.0, origin.y - 7.0, 16.0, MUTED);
    label(
        &format!("Grid: {spacing} m"),
        area.x + 12.0,
        area.y + 22.0,
        16.0,
        MUTED,
    );
}

pub(super) fn scene(
    world: &Simulation,
    camera: [f64; 2],
    zoom: f64,
    area: Rect,
    selection: (Option<usize>, Option<usize>),
    neighbors: bool,
    markers: &[Marker],
) {
    let (selected, selected_group) = selection;
    for zone in world.wind_zones() {
        draw_region(
            zone.area(),
            camera,
            zoom,
            area,
            Color::from_rgba(70, 180, 220, 255),
            "Wind",
        );
        let start = screen_point(zone.area().center(), camera, zoom, area);
        let v = zone.velocity();
        let p = zone.area().center();
        arrow(
            start,
            screen_point([p[0] + v[0], p[1] + v[1]], camera, zoom, area),
            ARRIVED,
        );
    }
    for zone in world.noise_zones() {
        draw_region(
            zone.area(),
            camera,
            zoom,
            area,
            Color::from_rgba(190, 130, 230, 255),
            &format!("Noise {:.3}", zone.position_stddev()),
        );
    }
    for &obstacle in world.obstacles() {
        draw_region(
            obstacle,
            camera,
            zoom,
            area,
            Color::from_rgba(240, 170, 90, 255),
            "Obstacle",
        );
    }
    for group in world.groups() {
        if let Command::Gather { center, radius } = group.command() {
            let p = screen_point(center, camera, zoom, area);
            let [r, g, b, a] = group.color();
            let tint = Color::from_rgba(r, g, b, a);
            draw_circle_lines(p.x, p.y, (radius * zoom) as f32, 1.0, tint);
            label(
                &format!("Group {} area", group.id()),
                p.x + 8.0,
                p.y + 20.0,
                16.0,
                tint,
            );
        }
    }
    if neighbors && let Some(id) = selected {
        let agent = world.agents()[id];
        if agent.state() != AgentState::Terminated {
            let p = screen_point(markers[id].position, camera, zoom, area);
            let radius = world.agent_config(id).neighbor_radius;
            draw_circle_lines(
                p.x,
                p.y,
                (radius * zoom) as f32,
                1.0,
                Color::from_rgba(106, 119, 75, 255),
            );
            for (j, _) in world.agents().iter().enumerate() {
                if world.perceives(id, j) {
                    let q = screen_point(markers[j].position, camera, zoom, area);
                    draw_line(p.x, p.y, q.x, q.y, 1.0, Color::from_rgba(98, 107, 76, 255));
                }
            }
        }
    }
    for (id, agent) in world.agents().iter().enumerate() {
        let p = screen_point(markers[id].position, camera, zoom, area);
        if !area.contains(p) {
            continue;
        }
        let [r, g, b, a] = world.group(agent.group_id()).unwrap().color();
        let tint = Color::from_rgba(r, g, b, a);
        match agent.state() {
            AgentState::Moving => draw_agent(p, markers[id].direction, tint),
            AgentState::Arrived => {
                draw_agent(p, markers[id].direction, tint);
                draw_circle_lines(p.x, p.y, 10.0, 1.0, ARRIVED);
            }
            AgentState::Terminated => {
                draw_line(p.x - 5.0, p.y - 5.0, p.x + 5.0, p.y + 5.0, 2.5, DEAD);
                draw_line(p.x - 5.0, p.y + 5.0, p.x + 5.0, p.y - 5.0, 2.5, DEAD);
            }
        }
        if selected_group == Some(agent.group_id()) && world.groups().len() > 1 {
            draw_circle_lines(p.x, p.y, 13.0, 1.5, SELECTED);
        }
        if selected == Some(id) {
            draw_circle_lines(p.x, p.y, 13.0, 2.0, SELECTED);
            label(&format!("#{id}"), p.x + 15.0, p.y - 10.0, 17.0, SELECTED);
        }
    }
}

pub(super) fn draw_region(
    shape: impl Into<Rectangle>,
    camera: [f64; 2],
    zoom: f64,
    area: Rect,
    color: Color,
    text: &str,
) {
    let corners = shape
        .into()
        .vertices()
        .map(|p| screen_point(p, camera, zoom, area));
    let fill = Color::new(color.r, color.g, color.b, 0.15);
    draw_triangle(corners[0], corners[1], corners[2], fill);
    draw_triangle(corners[0], corners[2], corners[3], fill);
    for i in 0..4 {
        let a = corners[i];
        let b = corners[(i + 1) % 4];
        draw_line(a.x, a.y, b.x, b.y, 2.0, color);
    }
    let x = corners.iter().map(|p| p.x).fold(f32::INFINITY, f32::min);
    let y = corners.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
    label(text, x + 6.0, y + 18.0, 16.0, color);
}

pub(super) fn route_lines(
    points: &[[f64; 2]],
    radius: f64,
    camera: [f64; 2],
    zoom: f64,
    area: Rect,
    color: Color,
) {
    let radius_px = (radius * zoom) as f32;
    let fill = Color::new(
        BG.r * 0.85 + color.r * 0.15,
        BG.g * 0.85 + color.g * 0.15,
        BG.b * 0.85 + color.b * 0.15,
        1.0,
    );
    // Paint the complete border first, then the interior so joins form one continuous channel.
    for (padding, tint) in [(2.0, color), (0.0, fill)] {
        for (i, point) in points.iter().enumerate() {
            let p = screen_point(*point, camera, zoom, area);
            if i > 0 {
                let prev = screen_point(points[i - 1], camera, zoom, area);
                draw_line(prev.x, prev.y, p.x, p.y, 2.0 * (radius_px + padding), tint);
            }
            draw_circle(p.x, p.y, radius_px + padding, tint);
        }
    }
    for (i, point) in points.iter().enumerate() {
        let p = screen_point(*point, camera, zoom, area);
        if i > 0 {
            let prev = screen_point(points[i - 1], camera, zoom, area);
            arrow(prev, p, color);
        }
        draw_circle(p.x, p.y, 3.0, color);
        label(&format!("{}", i + 1), p.x + 5.0, p.y - 5.0, 17.0, color);
    }
}

impl App {
    pub(super) fn draw_canvas(&mut self, ctx: FrameContext) {
        let FrameContext {
            area,
            mouse,
            on_canvas,
            ..
        } = ctx;
        grid(self.camera, self.zoom, area);
        for (id, route) in self.world.paths().iter().enumerate() {
            route_lines(
                route.points(),
                route.radius(),
                self.camera,
                self.zoom,
                area,
                if self.selected_path == Some(id) {
                    SELECTED
                } else {
                    ARRIVED
                },
            );
        }
        for group in self.world.groups() {
            if let Some(route) = group.route()
                && !self.world.paths().contains(route)
            {
                let [r, g, b, a] = group.color();
                route_lines(
                    route.points(),
                    route.radius(),
                    self.camera,
                    self.zoom,
                    area,
                    Color::from_rgba(r, g, b, a),
                );
            }
        }
        if !self.draft_route.is_empty() {
            let radius = self
                .route_width
                .trim()
                .parse::<f64>()
                .ok()
                .filter(|r| r.is_finite() && *r > 0.0)
                .map(|width| width * 0.5)
                .unwrap_or(1.0);
            route_lines(
                &self.draft_route,
                radius,
                self.camera,
                self.zoom,
                area,
                SELECTED,
            );
        }
        scene(
            &self.world,
            self.camera,
            self.zoom,
            area,
            (self.selected, self.selected_group),
            self.neighbors,
            &self.markers,
        );
        if let Some(object) = self.selected_object.and_then(|id| id.get(&self.world)) {
            object_outline(object.area(), self.camera, self.zoom, area);
        }
        if let Some(pending) = &self.pending_object {
            draw_region(
                pending.object.area(),
                self.camera,
                self.zoom,
                area,
                SELECTED,
                "Rotate, then click to place",
            );
            object_outline(pending.object.area(), self.camera, self.zoom, area);
        }
        if on_canvas {
            let point = world_point(mouse, self.camera, self.zoom, area);
            if let Some(draft) = &self.rectangle_draft
                && let Ok(rect) = draft.shape(point)
            {
                draw_region(
                    rect,
                    self.camera,
                    self.zoom,
                    area,
                    SELECTED,
                    "Height: click, then rotate",
                );
            }
            if self.tool == Tool::AddAgents
                && let Ok(radius) = self.settings.values[2].parse::<f64>()
                && radius.is_finite()
                && radius > 0.0
            {
                draw_circle_lines(mouse.x, mouse.y, (radius * self.zoom) as f32, 1.5, SELECTED);
                label(
                    "Click to add group",
                    mouse.x + 12.0,
                    mouse.y - 12.0,
                    16.0,
                    SELECTED,
                );
            }
        }
        if let Some((start, end)) = self.flight_guide {
            arrow(
                screen_point(start, self.camera, self.zoom, area),
                screen_point(end, self.camera, self.zoom, area),
                SELECTED,
            );
        }
        if let Some(start) = self.gesture
            && on_canvas
            && self.tool != Tool::Follow
        {
            let p = screen_point(start, self.camera, self.zoom, area);
            if self.tool == Tool::Fly {
                arrow(p, mouse, SELECTED);
                if let Ok(radius) = self.target_radius.trim().parse::<f64>()
                    && radius.is_finite()
                    && radius > 0.0
                {
                    draw_circle_lines(mouse.x, mouse.y, (radius * self.zoom) as f32, 1.0, SELECTED);
                }
            } else if self.tool == Tool::Rectangle {
                draw_line(p.x, p.y, mouse.x, p.y, 2.0, SELECTED);
                label(
                    "Width: release, then choose height",
                    p.x + 5.0,
                    p.y - 8.0,
                    16.0,
                    SELECTED,
                );
            } else if matches!(self.tool, Tool::Wind | Tool::Noise) {
                if let Ok(dimensions) = self.object_properties.drawn_dimensions(
                    self.tool,
                    start,
                    world_point(mouse, self.camera, self.zoom, area),
                ) && let Ok(object) =
                    self.object_properties
                        .create(self.tool, start, Some(dimensions))
                {
                    draw_region(
                        object.area(),
                        self.camera,
                        self.zoom,
                        area,
                        SELECTED,
                        "Size preview",
                    );
                }
            } else {
                draw_circle_lines(p.x, p.y, (mouse - p).length(), 1.5, SELECTED);
            }
        }
    }
}
