mod args;

use args::{ArduTapeArgs, Command};
use clap::Parser;
use std::path::PathBuf;

fn main() {
    let args = ArduTapeArgs::parse();

    // Select subcommand to run
    match args.operation {
        Command::Read { file_path } => read_tape(file_path),
        Command::Write { file_path } => write_tape(file_path),
    }
}

fn read_tape(file_path: PathBuf) {
    println!("Reading {}", file_path.display());
}

fn write_tape(file_path: PathBuf) {
    println!("Writing {}", file_path.display());
}
