use macroquad::prelude::*;
#[path = "viewer/mod.rs"]
mod viewer;
use viewer::{App, window_conf};

#[macroquad::main(window_conf)]
async fn main() {
    let mut app = match App::new() {
        Ok(app) => app,
        Err(error) => {
            eprintln!("viewer: {error}");
            return;
        }
    };
    loop {
        app.frame();
        next_frame().await;
    }
}
