use std::{error::Error, fmt};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimulationError {
    InvalidInput(&'static str),
    PlacementFailed,
    NumericOverflow,
}

impl fmt::Display for SimulationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidInput(message) => f.write_str(message),
            Self::PlacementFailed => f.write_str(
                "cannot place agents without overlap; reduce count or increase spawn radius",
            ),
            Self::NumericOverflow => {
                f.write_str("calculation exceeded the supported numerical range")
            }
        }
    }
}

impl Error for SimulationError {}
