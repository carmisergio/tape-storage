use std::fs::File;
use std::io;
use std::io::BufReader;
use std::io::Read;
use std::path::PathBuf;

pub fn get_blocks_from_file(file_path: PathBuf) -> Result<(Vec<[u8; 512]>, u32), String> {
    // Read raw bytes from file
    let data = match read_file(file_path) {
        Ok(data) => data,
        Err(err) => return Err(err),
    };

    let file_length: u32 = data.len().try_into().unwrap();

    let mut blocks: Vec<[u8; 512]> = vec![];
    let mut current_block: [u8; 512] = [0; 512];

    let mut current_byte_index = 0;

    // Process all bytes in the file
    for byte in data {
        current_block[current_byte_index] = byte;
        current_byte_index = current_byte_index + 1;

        if current_byte_index >= 512 {
            blocks.push(current_block);
            current_byte_index = 0;
        }
    }

    // If there were other bytes after the last push, do it now
    if current_byte_index != 0 {
        // Fill remaining part of current block
        for i in current_byte_index..512 {
            current_block[i] = 0;
        }

        blocks.push(current_block);
    }

    Ok((blocks, file_length))
}

fn read_file(file_path: PathBuf) -> Result<Vec<u8>, String> {
    let file = match File::open(file_path) {
        Ok(file) => file,
        Err(_) => return Err("Unable to open file".to_owned()),
    };

    let mut reader = BufReader::new(file);
    let mut read_buffer = Vec::new();

    // Read file into vector.
    match reader.read_to_end(&mut read_buffer) {
        Ok(_) => {}
        Err(_) => return Err("Unable to read from file".to_owned()),
    };

    Ok(read_buffer)
}
