use super::{
    ARRIVED, App, FOOTER, FrameContext, MUTED, PANEL, SELECTED, SURFACE, TEXT, Tool,
    objects::SceneObject, theme::label,
};
use macroquad::prelude::*;
use rust_playground::simulation::Simulation;

pub(super) fn inspector(world: &Simulation, selected: Option<usize>, x: f32) {
    draw_rectangle(x, 90.0, 250.0, 294.0, SURFACE);
    label("AGENT INSPECTOR", x + 16.0, 116.0, 18.0, MUTED);
    if let Some(id) = selected {
        let a = world.agents()[id];
        let p = a.position();
        let v = a.velocity();
        let air = a.air_velocity();
        let wind = a.wind_velocity();
        label(
            &format!("#{id} / G{} / {:?}", a.group_id(), a.state()),
            x + 16.0,
            146.0,
            18.0,
            TEXT,
        );
        label(
            &format!("Position m {:.3}, {:.3}", p[0], p[1]),
            x + 16.0,
            173.0,
            17.0,
            TEXT,
        );
        label(
            &world
                .measured_position(id)
                .map(|p| format!("Measured m {:.3}, {:.3}", p[0], p[1]))
                .unwrap_or_else(|_| "Measured: unavailable".to_owned()),
            x + 16.0,
            198.0,
            17.0,
            TEXT,
        );
        label(
            &format!("Ground m/s {:.3}, {:.3}", v[0], v[1]),
            x + 16.0,
            223.0,
            17.0,
            TEXT,
        );
        label(
            &format!("Air m/s {:.3}, {:.3}", air[0], air[1]),
            x + 16.0,
            248.0,
            17.0,
            TEXT,
        );
        label(
            &format!("Wind m/s {:.3}, {:.3}", wind[0], wind[1]),
            x + 16.0,
            273.0,
            17.0,
            TEXT,
        );
        let count = (0..world.agents().len())
            .filter(|&j| world.perceives(id, j))
            .count();
        let ground_speed = v[0].hypot(v[1]);
        label(
            &format!("{ground_speed:.2} m/s = {:.1} km/h", ground_speed * 3.6),
            x + 16.0,
            298.0,
            17.0,
            TEXT,
        );
        label(
            &format!("Visible agents  {count}"),
            x + 16.0,
            323.0,
            17.0,
            TEXT,
        );
        label(
            &format!("Body radius {:.3} m", world.agent_config(id).body_radius),
            x + 16.0,
            344.0,
            17.0,
            TEXT,
        );
        if let Some(route) = world.group(a.group_id()).and_then(|g| g.route()) {
            label(
                &format!(
                    "Waypoint {}/{}",
                    a.waypoint_index() + 1,
                    route.points().len()
                ),
                x + 16.0,
                362.0,
                16.0,
                MUTED,
            );
        } else {
            label("Esc: clear / F11: next", x + 16.0, 362.0, 16.0, MUTED);
        }
    } else {
        label(
            "Add a group, then inspect agents.",
            x + 16.0,
            151.0,
            18.0,
            TEXT,
        );
        label(
            "Group tab: Add [A], click field.",
            x + 16.0,
            179.0,
            17.0,
            MUTED,
        );
        label("F9 toggles neighbor lines.", x + 16.0, 207.0, 17.0, MUTED);
    }
}

pub(super) fn object_inspector(object: SceneObject, x: f32) {
    draw_rectangle(x, 90.0, 250.0, 294.0, SURFACE);
    label("SCENE OBJECT", x + 16.0, 116.0, 18.0, MUTED);
    let area = object.area();
    let center = area.center();
    let name = match object {
        SceneObject::Obstacle(_) => "Rectangle obstacle",
        SceneObject::Wind(_) => "Wind field",
        SceneObject::Noise(_) => "Measurement noise",
    };
    label(name, x + 16.0, 146.0, 19.0, TEXT);
    label(
        &format!("Center {:.2}, {:.2}", center[0], center[1]),
        x + 16.0,
        176.0,
        16.0,
        TEXT,
    );
    label(
        &format!("Width {:.2}", area.width()),
        x + 16.0,
        202.0,
        16.0,
        TEXT,
    );
    label(
        &format!("Length {:.2}", area.length()),
        x + 16.0,
        228.0,
        16.0,
        TEXT,
    );
    label(
        &format!("Angle {:.1} deg", area.angle().to_degrees()),
        x + 16.0,
        254.0,
        16.0,
        TEXT,
    );
    if let SceneObject::Wind(w) = object {
        label(
            &format!("Speed {:.3}", w.speed()),
            x + 16.0,
            280.0,
            16.0,
            TEXT,
        );
    }
    if let SceneObject::Noise(n) = object {
        label(
            &format!("Sigma {:.3}", n.position_stddev()),
            x + 16.0,
            280.0,
            16.0,
            TEXT,
        );
    }
    label("Drag round handle to rotate.", x + 16.0, 315.0, 15.0, MUTED);
    label("Scene: edit / Apply object.", x + 16.0, 341.0, 15.0, MUTED);
    label(
        "Alt-click: overlapping objects",
        x + 16.0,
        365.0,
        14.0,
        MUTED,
    );
}

impl App {
    pub(super) fn draw_overlays(&mut self, ctx: FrameContext) {
        let FrameContext { area, .. } = ctx;
        label("O Hovering = alive, waiting", 16.0, 741.0, 16.0, ARRIVED);
        if let Some(object) = self
            .pending_object
            .as_ref()
            .map(|p| p.object)
            .or_else(|| self.selected_object.and_then(|id| id.get(&self.world)))
        {
            object_inspector(object, screen_width() - 268.0);
        } else {
            inspector(&self.world, self.selected, screen_width() - 268.0);
        }
        if let Some(message) = self.error.as_ref().or(self.notice.as_ref()) {
            draw_rectangle(
                PANEL + 12.0,
                screen_height() - FOOTER - 78.0,
                area.w - 24.0,
                66.0,
                SURFACE,
            );
            label(
                if self.error.is_some() {
                    "! RUN PAUSED / RESTART TO APPLY VALID SETTINGS"
                } else {
                    "! COMMAND NOT APPLIED"
                },
                PANEL + 24.0,
                screen_height() - FOOTER - 53.0,
                17.0,
                SELECTED,
            );
            for (i, line) in message.as_bytes().chunks(85).take(2).enumerate() {
                label(
                    &String::from_utf8_lossy(line),
                    PANEL + 24.0,
                    screen_height() - FOOTER - 30.0 + i as f32 * 19.0,
                    16.0,
                    TEXT,
                );
            }
        }
        draw_rectangle(PANEL, screen_height() - FOOTER, area.w, FOOTER, SURFACE);
        label(
            if self.pending_object.is_some() {
                "Move the mouse to rotate; click to place. Esc: cancel. Apply object: use exact values."
            } else {
                match self.tool {
                    Tool::AddAgents => {
                        "Add: edit the new-group profile, then click a spawn center."
                    }
                    Tool::Inspect => {
                        "Inspect: click agent or object. Drag the round handle to rotate. Alt-click cycles objects."
                    }
                    Tool::Fly => {
                        "Fly: drag towards a preferred area for the selected group. The tool finishes on release."
                    }
                    Tool::Gather => {
                        "Gather: draw a soft area. Agents take nearby free places; the group shape is unrestricted."
                    }
                    Tool::Spread => {
                        "Spread: choose Neighbor gap (<= View radius), then click / drag. No target circle."
                    }
                    Tool::Follow => {
                        "Follow: hold left mouse over the field. Agents follow a target region."
                    }
                    Tool::Rectangle => {
                        "Rectangle: drag width, release; click height; rotate and click to place."
                    }
                    Tool::Wind => {
                        "Wind: choose width, length, angle and speed. Click / drag, rotate, then click to place."
                    }
                    Tool::Noise => {
                        "Noise: choose width, length, angle and sigma. Click / drag, rotate, then click to place."
                    }
                    Tool::Route => {
                        "Route: click vertices; Enter saves and assigns to the selected group. Assign saved paths in Swarm."
                    }
                }
            },
            PANEL + 14.0,
            screen_height() - 37.0,
            16.0,
            MUTED,
        );
        label(
            "Pan: right-drag / arrows   Zoom: wheel / +/-   F10: reset view   Esc: inspect",
            PANEL + 14.0,
            screen_height() - 15.0,
            16.0,
            MUTED,
        );
    }
}
