use clap::{Parser, Subcommand};
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
    
}
