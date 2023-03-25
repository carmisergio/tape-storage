mod args;
mod file;
mod serial;
mod ui;

use file::{get_blocks_from_file, write_blocks_to_file};
use serial::{open_serial_connection, SerialConnectError, SerialPort};
use ui::{get_progress_bar, get_spinner, wait_for_enter};

use args::{ArduTapeArgs, Command};
use clap::Parser;
use colored::Colorize;
use indicatif::HumanDuration;
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;

const HEADER_TIME: Duration = Duration::from_millis(4600); // mS // Including first recal
const BLOCK_TIME: Duration = Duration::from_millis(6212); // mS

fn main() {
    let args = ArduTapeArgs::parse();

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
        "{} to ArduTape on port {}",
        "Connected".bright_green(),
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
    println!("Reading into {}", &file_path.display());
    println!("Press [{}] on tape.", "PLAY".bright_cyan());

    // Calibration wait spinner
    let calibration_spinner = get_spinner();
    calibration_spinner.set_message("Waiting for calibration tone...");

    // Start read on interface
    send_read_command(serial_port).expect("Unable to complete read command");

    // Wait for read begin command
    match wait_for_command_or_abort(serial_port, b'B') {
        Ok(_) => {}
        Err(_) => {
            println!("{}", "Read aborted!".bright_red());
            return;
        }
    };

    // Finish calibration spinner and start header read spinner
    calibration_spinner.finish();
    let header_read_spinner = get_spinner();
    header_read_spinner.set_message("Reading header...");

    // Wait for header read
    match wait_for_command_or_abort(serial_port, b'M') {
        Ok(_) => {}
        Err(_) => {
            println!("{}", "Read error!".bright_red());
            return;
        }
    };

    // Read file length from interface
    let file_length = serial_read_u32(serial_port).expect("Unable to read thing");
    let blocks_to_read: u32 = (file_length + 512 - 1) / 512;

    // Compute eta
    let read_eta = get_read_eta(blocks_to_read.try_into().unwrap());

    // Finish calibration spinner
    header_read_spinner.finish();

    println!("{}", "Got header!".bright_green());
    println!(" - File length: {file_length} B");
    println!(" - Blocks: {blocks_to_read}");
    println!(" - ETA: {}", HumanDuration(read_eta).to_string());

    let mut blocks_read: u32 = 0;
    let mut blocks: Vec<[u8; 512]> = vec![];

    // Start progres bar
    let progress_bar = get_progress_bar(blocks_to_read.try_into().unwrap());
    progress_bar.set_message("Reading data...");
    progress_bar.reset();
    progress_bar.set_position(0);
    progress_bar.enable_steady_tick(Duration::from_millis(100));

    // Main read loop
    while blocks_read < blocks_to_read {
        // Wait for data available command
        match wait_for_command_or_abort(serial_port, b'D') {
            Ok(_) => {}
            Err(_) => {
                println!("{}", "Read error!".bright_red());
                return;
            }
        };

        let block = serial_read_block(serial_port).unwrap();
        // print_block(&block);

        // Add block to blocks
        blocks.push(block);

        blocks_read += 1;

        // Set progress bar position
        progress_bar.set_position(blocks_read.try_into().unwrap());
    }

    // Finish progress bar
    progress_bar.finish();

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

    println!(
        "{} Press [{}] on tape.",
        "Read success!".bright_green(),
        "STOP".bright_white()
    );
}

// fn print_block(block: &[u8; 512]) {
//     let mut counter = 0;
//     for byte in block {
//         if counter >= 32 {
//             println!();
//             counter = 0;
//         }
//         if byte <= &0xF {
//             print!("0");
//         }
//         print!("{:X} ", byte);
//         counter += 1;
//     }
//     println!();
// }

fn write_tape(serial_port: &mut Box<dyn SerialPort>, file_path: PathBuf) {
    // Read blocks from file
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

    let write_duration = get_write_eta(blocks.len());

    println!("Writing {}", &file_path.display());
    println!(" - File length: {} B", file_length);
    println!(" - Blocks: {}", blocks.len());
    println!(" - ETA: {}", HumanDuration(write_duration).to_string());

    print!(
        "Press [{}] on tape, then press [ENTER] to continue...",
        "RECORD".bright_red()
    );
    wait_for_enter();

    // Header write spinner
    let header_write_spinner = get_spinner();
    header_write_spinner.set_message("Writing header...");

    send_write_command(serial_port, file_length).expect("Unable to complete write command");

    // Data write progress bar
    let progress_bar = get_progress_bar(blocks.len());

    // Main write loop
    let mut block_to_write: usize = 0;
    let mut read_buf: [u8; 1] = [0];

    while block_to_write < blocks.len() {
        match serial_port.read(&mut read_buf) {
            Ok(_) => {}
            // No data was provided
            Err(_) => {
                sleep(Duration::from_millis(10));
                continue;
            }
        };

        // Write block to serial
        if read_buf[0] == b'D' {
            // println!("Sending block: {block_to_write}");
            serial_write_block(serial_port, &blocks[block_to_write]).unwrap();
            block_to_write = block_to_write + 1;

            // Finish spinner if still going
            if !header_write_spinner.is_finished() {
                header_write_spinner.finish();
                progress_bar.enable_steady_tick(Duration::from_millis(100));
                progress_bar.set_message("Writing data...");
            }

            // Set position on progress bar
            progress_bar.set_position((block_to_write - 1).try_into().unwrap());
        }
    }

    // Wait for SUCCESS command
    wait_for_command(serial_port, b'S');

    progress_bar.finish();
    println!(
        "{} Press [{}] on tape.",
        "Write success!".bright_green(),
        "STOP".bright_white()
    );
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

fn get_write_eta(blocks: usize) -> Duration {
    let blocks_ms: usize = BLOCK_TIME.as_millis().try_into().unwrap();
    HEADER_TIME + Duration::from_millis((blocks_ms * blocks).try_into().unwrap())
}

fn get_read_eta(blocks: usize) -> Duration {
    let blocks_ms: usize = BLOCK_TIME.as_millis().try_into().unwrap();
    Duration::from_millis((blocks_ms * blocks).try_into().unwrap())
}
