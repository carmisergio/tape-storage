mod args;
mod file;
mod serial;

use file::get_blocks_from_file;
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

    let (blocks, file_length) = get_blocks_from_file(file_path).unwrap();

    // println!("{:?}", blocks);

    send_write_command(serial_port, file_length).expect("Unable to complete write command");

    let mut block_to_write: usize = 0;

    let mut read_buf: [u8; 1] = [0];

    // Main write loop
    while block_to_write < blocks.len() {
        match serial_port.read(&mut read_buf) {
            Ok(_) => {}
            Err(_) => continue,
        };
        if read_buf[0] == b'D' {
            println!("Sending block: {block_to_write}");
            serial_write_block(serial_port, &blocks[block_to_write]).unwrap();
            block_to_write = block_to_write + 1;
        }
    }
}

fn send_write_command(serial_port: &mut Box<dyn SerialPort>, file_length: u32) -> Result<(), ()> {
    // Send write command
    match serial_write_u8(serial_port, b'W') {
        Ok(_) => {}
        Err(_) => return Err(()),
    };

    // Send write command
    match serial_write_u32(serial_port, file_length) {
        Ok(_) => {}
        Err(_) => return Err(()),
    };

    Ok(())
}

fn serial_write_u32(serial_port: &mut Box<dyn SerialPort>, number: u32) -> Result<(), ()> {
    let number_as_bytes = number.to_le_bytes();
    match serial_port.write(&number_as_bytes) {
        Ok(_) => Ok(()),
        Err(_) => Err(()),
    }
}

fn serial_write_u8(serial_port: &mut Box<dyn SerialPort>, character: u8) -> Result<(), ()> {
    let serial_buffer: [u8; 1] = [character];
    match serial_port.write(&serial_buffer) {
        Ok(_) => Ok(()),
        Err(_) => Err(()),
    }
}

fn serial_write_block(serial_port: &mut Box<dyn SerialPort>, block: &[u8; 512]) -> Result<(), ()> {
    match serial_port.write(block) {
        Ok(_) => Ok(()),
        Err(_) => Err(()),
    }
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
