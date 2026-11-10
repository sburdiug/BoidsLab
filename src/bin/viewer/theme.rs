use super::{BG, MINT, MUTED, TEXT};
use macroquad::{
    prelude::*,
    ui::{Skin, root_ui, widgets},
};

pub(crate) fn window_conf() -> Conf {
    Conf {
        window_title: "BoidsLab".into(),
        window_width: 1200,
        window_height: 820,
        window_resizable: true,
        ..Default::default()
    }
}

pub(super) fn label(text: &str, x: f32, y: f32, size: f32, color: Color) {
    draw_text(text, x, y, size, color);
}

pub(super) fn button(
    text: &str,
    x: f32,
    y: f32,
    width: f32,
    enabled: bool,
    selected: bool,
) -> bool {
    if !enabled {
        draw_rectangle(x, y, width, 36.0, Color::from_rgba(29, 40, 55, 255));
        label(text, x + 10.0, y + 23.0, 18.0, MUTED);
        return false;
    }
    widgets::Button::new(text)
        .position(vec2(x, y))
        .size(vec2(width, 36.0))
        .selected(selected)
        .ui(&mut root_ui())
}

pub(super) fn skin() -> Skin {
    let ui = root_ui();
    let button_style = ui
        .style_builder()
        .font_size(18)
        .text_color(TEXT)
        .text_color_hovered(TEXT)
        .text_color_clicked(BG)
        .color(Color::from_rgba(37, 53, 72, 255))
        .color_hovered(Color::from_rgba(51, 73, 96, 255))
        .color_clicked(MINT)
        .color_selected(Color::from_rgba(37, 78, 97, 255))
        .color_selected_hovered(Color::from_rgba(47, 91, 109, 255))
        .build();
    let editbox_style = ui
        .style_builder()
        .font_size(18)
        .text_color(TEXT)
        .color(BG)
        .color_clicked(Color::from_rgba(31, 50, 71, 255))
        .color_selected(Color::from_rgba(50, 86, 121, 255))
        .build();
    Skin {
        button_style,
        editbox_style,
        ..ui.default_skin()
    }
}
