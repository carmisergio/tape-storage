use std::fs::File;
pub use std::io::Error;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::PathBuf;

pub fn get_blocks_from_file(file_path: &PathBuf) -> Result<(Vec<[u8; 512]>, u32), io::Error> {
    // Read raw bytes from file
    let data = read_file(file_path.to_path_buf())?;

    // Get file length
    let file_length: u32 = match data.len().try_into() {
        Ok(number) => number,
        Err(_) => return Err(io::Error::from(io::ErrorKind::Other)),
    };

    // Allocate spce for data
    let mut blocks: Vec<[u8; 512]> = vec![];
    let mut current_block: [u8; 512] = [0; 512];

    let mut current_byte_index = 0;

    // Process all bytes in the file
    for byte in data {
        // Put current byte in block
        current_block[current_byte_index] = byte;
        current_byte_index = current_byte_index + 1;
        // If at the end of the current block
        if current_byte_index >= 512 {
            // Add block to blocks
            blocks.push(current_block);
            // Start over in current block
            current_byte_index = 0;
        }
    }

    // If there were other bytes after the last push, finish processing last
    // block
    if current_byte_index != 0 {
        // Fill remaining part of current block with 0
        for i in current_byte_index..512 {
            current_block[i] = 0;
        }
        // Add block to blocks
        blocks.push(current_block);
    }

    Ok((blocks, file_length))
}

fn read_file(file_path: PathBuf) -> Result<Vec<u8>, io::Error> {
    // Open file
    let file = File::open(file_path)?;

    let mut reader = BufReader::new(file);
    let mut read_buffer = Vec::new();

    // Read file into vector.
    reader.read_to_end(&mut read_buffer)?;

    Ok(read_buffer)
}

pub fn write_blocks_to_file(
    file_path: &PathBuf,
    blocks: &Vec<[u8; 512]>,
    file_length: u32,
) -> Result<(), io::Error> {
    let mut bytes_inserted: u32 = 0;
    let mut data: Vec<u8> = vec![];

    // Add bytes from blocks to data buffer
    for block in blocks {
        for byte in block {
            // If we've reached the end of file, don't add any more bytes
            if bytes_inserted >= file_length {
                break;
            }

            // If we have not yet reached the end of file, add block to file
            data.push(*byte);
            bytes_inserted += 1;
        }
    }

    // Write data buffer to file
    write_file(file_path.to_path_buf(), &data)?;

    Ok(())
}

fn write_file(file_path: PathBuf, data: &Vec<u8>) -> Result<(), io::Error> {
    // Open file if it exists, create if it doesn't
    let file = File::create(file_path)?;

    let mut writer = BufWriter::new(file);
    writer.write_all(data)?;

    Ok(())
}
