use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Debug, Parser)]
#[clap(author, version, about)]
pub struct ArduTapeArgs {
    #[clap(subcommand)]
    pub operation: Command,
}

#[derive(Debug, Subcommand)]
pub enum Command {
    /// Read from tape
    Read {
        /// File to read from tape
        file_path: PathBuf,
    },
    /// Write to tape
    Write {
        /// File to read from tape
        file_path: PathBuf,
    },
}
