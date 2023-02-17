use serialport::SerialPort;
use std::io;
use std::thread::sleep;
use std::time::{Duration, Instant};

// Config constants
const CONFIG_BAUD_RATE: u32 = 38400;
const CONFIG_HANDSHAKE_COMMAND: &[u8] = b"H";
const CONFIG_HANDSHAKE_EXPECTED_RESPONSE: &[u8] = "AT-1.0.0\n".as_bytes();
const CONFIG_HANDSHAKE_READ_TIMEOUT: u64 = 1000;
const CONFIG_HANDSHAKE_RETRY_DELAY: u64 = 1000;

#[derive(Debug)]
pub enum SerialConnectError {
    PortOpenError,
    PortWriteError,
    PortReadError,
    PortReadTimedOut,
    GetAvailablePortsError,
    NoAvailablePorts,
    NoArduTapeDetected,
    InvalidHandshakeResponse,
}

pub fn open_serial_connection(
    device: Option<String>,
) -> Result<Box<dyn SerialPort>, SerialConnectError> {
    match device {
        Some(device) => match connect_and_handshake(device) {
            Ok(port) => Ok(port),
            // Err(_) => Err(&format!("Unable to open serial port: {}", device)),
            Err(err) => Err(err),
        },
        None => find_and_open(),
    }
}

pub fn read_n_bytes(
    port: &mut Box<dyn SerialPort>,
    bytes_to_read: usize,
    timeout: Duration,
) -> Result<Vec<u8>, SerialConnectError> {
    let mut result_buf: Vec<u8> = vec![];
    let mut read_buf: Vec<u8> = vec![0; 8];
    let mut bytes_read: usize = 0;

    // Save start time
    let start_time = Instant::now();

    // Read until we have read the amount of bytes requested
    while bytes_read < bytes_to_read {
        // Check timeout
        if start_time.elapsed() > timeout {
            return Err(SerialConnectError::PortReadTimedOut);
        }

        match port.read(read_buf.as_mut_slice()) {
            Ok(nbytes) => {
                result_buf.append(&mut Vec::from(&read_buf[0..nbytes]));
                bytes_read += nbytes;
            }
            Err(ref err) => {
                if err.kind() != io::ErrorKind::TimedOut {
                    return Err(SerialConnectError::PortReadError);
                }
            }
        }
    }
    Ok(result_buf)
}

// TODO WAS NOT RETRYING HANDSHAKE
fn connect_and_handshake(port_name: String) -> Result<Box<dyn SerialPort>, SerialConnectError> {
    let mut port = match open_serial_port(&port_name) {
        Ok(port) => port,
        Err(_) => return Err(SerialConnectError::PortOpenError),
    };

    // Immediately try handshaking
    match check_connection(&mut port) {
        Ok(_) => return Ok(port),
        Err(_) => {}
    };

    // If it doesn't work, wait for a while and retry
    sleep(Duration::from_millis(CONFIG_HANDSHAKE_RETRY_DELAY));
    match check_connection(&mut port) {
        Ok(_) => Ok(port),
        Err(err) => Err(err),
    }
}

fn open_serial_port(port_name: &String) -> serialport::Result<Box<dyn SerialPort>> {
    serialport::new(port_name, CONFIG_BAUD_RATE)
        .timeout(Duration::from_millis(10))
        .open()
}

fn check_connection(port: &mut Box<dyn SerialPort>) -> Result<(), SerialConnectError> {
    // Clear input buffer
    match port.clear(serialport::ClearBuffer::Input) {
        Ok(()) => {}
        Err(_) => {}
    }

    // Send hello command
    match port.write(CONFIG_HANDSHAKE_COMMAND) {
        Ok(_) => {}
        Err(_) => return Err(SerialConnectError::PortWriteError),
    }

    // Get response
    let response = match read_n_bytes(
        port,
        8,
        Duration::from_millis(CONFIG_HANDSHAKE_READ_TIMEOUT),
    ) {
        Ok(response) => response,
        Err(err) => return Err(err),
    };

    // Check response
    if response != CONFIG_HANDSHAKE_EXPECTED_RESPONSE {
        return Err(SerialConnectError::InvalidHandshakeResponse);
    }

    Ok(())
}

fn find_and_open() -> Result<Box<dyn SerialPort>, SerialConnectError> {
    let ports = match serialport::available_ports() {
        Ok(ports) => ports,
        Err(..) => return Err(SerialConnectError::GetAvailablePortsError),
    };

    if ports.len() < 1 {
        return Err(SerialConnectError::NoAvailablePorts);
    };

    // Loop over all found ports
    for port_name in ports {
        let mut port = match open_serial_port(&port_name.port_name) {
            Ok(port) => port,
            Err(_) => continue,
        };
        match check_connection(&mut port) {
            Ok(()) => return Ok(port),
            Err(_) => continue,
        }
    }

    Err(SerialConnectError::NoArduTapeDetected)
}
