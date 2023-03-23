mod args;
mod file;
mod serial;

use file::{get_blocks_from_file, write_blocks_to_file};
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
        Command::Read { file_path } => read_tape(&mut serial_port, file_path),
        Command::Write { file_path } => write_tape(&mut serial_port, file_path),
    }
}

fn read_tape(serial_port: &mut Box<dyn SerialPort>, file_path: PathBuf) {
    println!("Reading into {}", file_path.display());

    // Start read on interface
    send_read_command(serial_port).expect("Unable to complete read command");

    // Wait for read begin command
    match wait_for_command_or_abort(serial_port, b'B') {
        Ok(_) => {}
        Err(_) => {
            println!("Read aborted!");
            return;
        }
    };

    // Read file length from interface
    let file_length = serial_read_u32(serial_port).expect("Unable to read thing");

    println!("Calibration succesful!");
    println!("File length: {file_length}");

    let mut blocks_read: u32 = 0;
    let blocks_to_read: u32 = (file_length + 512 - 1) / 512;
    let mut blocks: Vec<[u8; 512]> = vec![];

    // Main read loop
    while blocks_read < blocks_to_read {
        // Wait for data available command
        match wait_for_command_or_abort(serial_port, b'D') {
            Ok(_) => {}
            Err(_) => {
                println!("Read aborted!");
                return;
            }
        };

        println!("Reading block: {blocks_read}");
        let block = serial_read_block(serial_port).unwrap();
        // print_block(&block);

        // Add block to blocks
        blocks.push(block);

        blocks_read += 1;
    }
    println!("Read done!");
    // println!("{:?}", blocks);

    println!("Writing data to file...");
    match write_blocks_to_file(&file_path, &blocks, file_length) {
        Ok(_) => {}
        Err(err) => {
            println!(
                "{}: Couldn't write file {:?}: {}",
                "Error".bright_red(),
                &file_path.display(),
                err,
            );
            return;
        }
    };

    println!("Done!");
}

fn print_block(block: &[u8; 512]) {
    let mut counter = 0;
    for byte in block {
        if counter >= 32 {
            println!();
            counter = 0;
        }
        if byte <= &0xF {
            print!("0");
        }
        print!("{:X} ", byte);
        counter += 1;
    }
    println!();
}

fn write_tape(serial_port: &mut Box<dyn SerialPort>, file_path: PathBuf) {
    println!("Writing {}", &file_path.display());

    let (blocks, file_length) = match get_blocks_from_file(&file_path) {
        Ok(data) => data,
        Err(err) => {
            println!(
                "{}: Couldn't read file {:?}: {}",
                "Error".bright_red(),
                &file_path.display(),
                err,
            );
            return;
        }
    };

    println!("Blocks to write: {}", blocks.len());

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

    // Wait for SUCCESS command
    wait_for_command(serial_port, b'S');

    println!("Done!");
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

fn send_read_command(serial_port: &mut Box<dyn SerialPort>) -> Result<(), ()> {
    // Send write command
    match serial_write_u8(serial_port, b'R') {
        Ok(_) => {}
        Err(_) => return Err(()),
    };
    Ok(())
}

fn wait_for_command(serial_port: &mut Box<dyn SerialPort>, command: u8) {
    let mut read_buf: [u8; 1] = [0];

    loop {
        match serial_port.read(&mut read_buf) {
            Ok(_) => {}
            Err(_) => continue,
        }

        if read_buf[0] == command {
            return;
        }
    }
}

enum WaitForCommandError {
    // TimeoutError,
    Aborted,
}

fn wait_for_command_or_abort(
    serial_port: &mut Box<dyn SerialPort>,
    command: u8,
) -> Result<(), WaitForCommandError> {
    let mut read_buf: [u8; 1] = [0];

    loop {
        match serial_port.read(&mut read_buf) {
            Ok(_) => {}
            Err(_) => continue,
        }

        if read_buf[0] == command {
            return Ok(());
        } else if read_buf[0] == b'A' {
            return Err(WaitForCommandError::Aborted);
        }
    }
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

fn serial_read_u32(serial_port: &mut Box<dyn SerialPort>) -> Result<u32, ()> {
    let mut number_as_bytes: [u8; 4] = [0; 4];
    let mut bytes_read = 0;

    // Read bytes from serial
    while bytes_read < 4 {
        match serial_port.read(&mut number_as_bytes[bytes_read..4]) {
            Ok(bytes) => bytes_read -= bytes,
            Err(_) => continue,
        }
    }

    // Convert bytes to u32
    let number = u32::from_le_bytes(number_as_bytes);

    Ok(number)
}

fn serial_read_block(serial_port: &mut Box<dyn SerialPort>) -> Result<[u8; 512], ()> {
    let mut block: [u8; 512] = [0; 512];
    let mut bytes_read = 0;

    // Read bytes from serial
    while bytes_read < 512 {
        match serial_port.read(&mut block[bytes_read..512]) {
            Ok(bytes) => bytes_read += bytes,
            Err(_) => continue,
        }
    }

    Ok(block)
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
