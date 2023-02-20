mod args;
mod serial;

use serial::{open_serial_connection, SerialConnectError, SerialPort};

use args::{ArduTapeArgs, Command};
use clap::Parser;
use colored::Colorize;
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;

fn main() {
    let args = ArduTapeArgs::parse();

    // Open serial port
    let mut serial_port = match open_serial_connection(args.device) {
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

    sleep(Duration::from_millis(200));

    // Select subcommand to run
    match args.operation {
        Command::Read { file_path } => read_tape(file_path),
        Command::Write { file_path } => write_tape(&mut serial_port, file_path),
    }
}

fn read_tape(file_path: PathBuf) {
    println!("Reading {}", file_path.display());
    // test_serial();
}

fn write_tape(serial_port: &mut Box<dyn SerialPort>, file_path: PathBuf) {
    println!("Writing {}", file_path.display());
    send_write_command(serial_port, 1500);
}

fn send_write_command(serial_port: &mut Box<dyn SerialPort>, file_length: u32) {
    let write_buf: &mut [u8; 5] = &mut [0; 5];
    // Set write command
    write_buf[0] = b'W';

    // Convert file length to bytes
    let number_as_bytes = file_length.to_le_bytes();

    // Copy file length into write buffer
    for i in 0..4 {
        write_buf[i + 1] = number_as_bytes[i];
    }

    println!("{:?}", write_buf);

    serial_port
        .write(write_buf)
        .expect("Couldn't write to serial port!");

    sleep(Duration::from_millis(1000));
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
