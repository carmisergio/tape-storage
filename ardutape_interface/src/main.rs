mod args;
mod serial;

use serial::{open_serial_connection, SerialConnectError};

use args::{ArduTapeArgs, Command};
use clap::Parser;
use colored::Colorize;
use std::path::PathBuf;

fn main() {
    let args = ArduTapeArgs::parse();

    // Open serial port
    let serial_port = match open_serial_connection(args.device) {
        Ok(port) => port,
        Err(err) => {
            println!(
                "{} {}",
                "Error:".bright_red(),
                get_serial_open_error_message(err),
            );
            return;
        }
    };

    println!(
        "Connected to ArduTape on port {}",
        serial_port.name().unwrap()
    );

    // // Select subcommand to run
    // match args.operation {
    //     Command::Read { file_path } => read_tape(file_path)
    //     Command::Write { file_path } => write_tape(file_path),
    // }
}

fn read_tape(file_path: PathBuf) {
    println!("Reading {}", file_path.display());
    // test_serial();
}

fn write_tape(file_path: PathBuf) {
    println!("Writing {}", file_path.display());
}

fn get_serial_open_error_message(err: SerialConnectError) -> &'static str {
    match err {
        SerialConnectError::PortOpenError => "Couldn't open serial port!",
        SerialConnectError::PortWriteError => "Couldn't write to serial port!",
        SerialConnectError::PortReadError => "Couldn't read from serial port!",
        SerialConnectError::PortReadTimedOut => "Serial port read timed out!",
        SerialConnectError::GetAvailablePortsError => "Couldn't get available serial ports!",
        SerialConnectError::NoAvailablePorts => "No serial ports available!",
        SerialConnectError::NoArduTapeDetected => "Couldn't find ArduTape interface!",
        SerialConnectError::InvalidHandshakeResponse => "Invalid handshake response!",
    }
}
