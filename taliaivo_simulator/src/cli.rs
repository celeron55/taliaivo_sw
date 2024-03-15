use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
#[command(version)]
pub struct Cli {
    /// Arena side length
    #[arg(short, long, default_value_t = 125.0)]
    pub arena_size: f32,

    /// Replay log file
    #[arg(short, long, value_name = "FILE")]
    pub replay: Option<PathBuf>,

    /// Enable primary robot weapon
    #[arg(short = 'w', long, default_value_t = false)]
    pub enable_primary_robot_weapon: bool,

    /// Enable secondary robot weapon
    #[arg(short = 's', long, default_value_t = false)]
    pub enable_secondary_robot_weapon: bool,

    /// Enable secondary robot
    #[arg(short = 'S', long, default_value_t = false)]
    pub enable_secondary_robot: bool,

    /// Enable secondary robot brain
    #[arg(short = 'B', long, default_value_t = false)]
    pub enable_secondary_robot_brain: bool,

    /// Drive primary robot using keyboard
    #[arg(short = 'k', long, default_value_t = false)]
    pub primary_robot_keyboard: bool,
}
