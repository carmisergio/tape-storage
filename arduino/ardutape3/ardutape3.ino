


// Feature enables
//#define DEBUG
#define EXT_TRIG

#ifdef DEBUG
#include <SoftwareSerial.h>
#endif

// Pin configuration
#define DATA_WRITE_PIN 4
#define DATA_READ_PIN 3
#define EXT_TRIGGER_PIN 12

// Serial configuration
#define MAIN_SERIAL_SPEED 38400
#define DEBUG_SERIAL_SPEED 115200
#define DEBUG_SERIAL_RX 12
#define DEBUG_SERIAL_TX 11
#define BLOCK_READ_BYTE_TIMEOUT 500 // ms

// Buffer sizes
#define BLOCK_SIZE 512
#define HEADER_SIZE 32
#define CRC_SIZE 2
#define DATA_BUF_SIZE BLOCK_SIZE + CRC_SIZE
#define HEADER_BUF_SIZE HEADER_SIZE + CRC_SIZE

// Calibration config
#define CAL_SUCCESSFUL_PULSES 550
#define RECAL_SUCCESSFUL_PULSES 100
#define CAL_ACCEPTED_VARIATION 0.15 // 15% accepted variation in calibration timing
#define CAL_VALID_MAX 4000 // uS
#define CAL_VALID_MIN 500 // uS
#define CONTINUOUS_CAL_WEIGHT 5 // Weight of old calibration value in respect to new
#define RECAL_TIMEOUT 1000 // ms

// Tape identification string
#define TAPE_IDENT_STR_LEN 8
const char TAPE_IDENT_STR[]  = "VCS02SER";

// Data encoding defines
#define LEADER_LENGTH 3000
#define RECAL_LEADER_LENGTH 1000

// Commands
#define CMD_HANDSHAKE 'H'
#define CMD_READ 'R'
#define CMD_WRITE 'W'
#define CMD_DATA_REQUEST 'D'
#define CMD_ABORT 'A'
#define CMD_SUCCESS 'S'
#define CMD_BEGIN 'B'

// Data buffers
volatile byte header_buf[HEADER_BUF_SIZE];
volatile byte data_buf[DATA_BUF_SIZE]; 

// Software serial for debug
#ifdef DEBUG
SoftwareSerial DebugSerial(DEBUG_SERIAL_RX, DEBUG_SERIAL_TX);
#endif

/************************   Read functions   ************************/
volatile int read_phase; // Read phase -> 0: not reading, 1: initial calibration, 2: leader wait, 3: header, 4: intermediate calibration, 5: leader wait, 6: data
volatile bool first_edge;
volatile uint32_t current_time;
volatile uint32_t last_edge_time;
volatile uint16_t current_time_delta;
volatile uint32_t cal_sum;
volatile int cal_count;
volatile uint16_t cal_value;
volatile unsigned long long calibration_start;
volatile byte edge_count;

volatile bool read_aborted;

// Data read edge change interrupt
// ICACHE_RAM_ATTR 
void read_edge_isr() {

  // Get time of current edge
  current_time = micros();

  // Calculate time delta of current transition
  current_time_delta = current_time - last_edge_time;

  if(read_phase != 0) {
    // See what phase we're in
    if(read_phase == 1) {
      // Calibration phase
      if(!first_edge) {

        // DebugSerial.println(current_time_delta);

        if(current_time_delta <= CAL_VALID_MAX && current_time_delta >= CAL_VALID_MIN) {

          // Check if current time is within acceptable deviation
          if(current_time_delta > cal_value + cal_value * CAL_ACCEPTED_VARIATION || current_time_delta < cal_value - cal_value * CAL_ACCEPTED_VARIATION) {
            // This sequence is not the file leader
            // Start over
            cal_sum = 0;
            cal_count = 0;
          }

          // Add current transition time to sum
          cal_sum += current_time_delta;
          cal_count++;

          // Compute average
          cal_value = cal_sum / cal_count;
            
          // If there were enough pulses to succesfully calibrate
          if(cal_count >= CAL_SUCCESSFUL_PULSES) {
            
            // Set read phase 2 variables
            edge_count = 0;
            first_edge = true;

            // Go to next read phase
            read_phase = 2;
          }
        } else {
          // If value was outside range of values
          // Start over
          cal_sum = 0;
          cal_count = 0;
        }

        

      } else {
        // It's not the first edge anymore
        first_edge = false;
      }


    } else if(read_phase == 4) {
      // Recalibration phase
      if(!first_edge) {

        if(current_time_delta <= CAL_VALID_MAX && current_time_delta >= CAL_VALID_MIN) {

          // Check if current time is within acceptable deviation
          if(current_time_delta > cal_value + cal_value * CAL_ACCEPTED_VARIATION || current_time_delta < cal_value - cal_value * CAL_ACCEPTED_VARIATION) {
            // This sequence is not the file leader
            // Start over
            cal_sum = 0;
            cal_count = 0;
          }

          // Add current transition time to sum
          cal_sum += current_time_delta;
          cal_count++;

          // Compute average
          cal_value = cal_sum / cal_count;
            
          // If there were enough pulses to succesfully calibrate
          if(cal_count >= RECAL_SUCCESSFUL_PULSES) {
            
            // Set read phase 2 variables
            edge_count = 0;
            first_edge = true;

            // Go to next read phase
            read_phase = 5;
          }
        } else {
          // If value was outside range of values
          // Start over
          cal_sum = 0;
          cal_count = 0;
        }

        // Check if time is outside timeout  (hasn't been able to recalibrate)
        // if(millis() - calibration_start > RECAL_TIMEOUT) {
        //   // Communicate to main that read has been aborted
        //   read_aborted = true;

        //   // Stop read
        //   read_phase = 0;  
        // }

        

      } else {
        // It's not the first edge anymore
        first_edge = false;
      }
    } else {
      if(!first_edge) {
        // If we're on at least the second edge
        if(edge_count == 0) {

          // Check time with calibrated threashold value
          if(current_time_delta > ((cal_value * 13) / 20)) {

            // Received bit 1
            process_bit(1);

            // Add data point to continuous calibration
            // continuous_cal(current_time_delta);

          } else {

            // Received bit 0
            process_bit(0);  

            // Add data point to continuous calibration
            // continuous_cal(current_time_delta << 1); // Transition for a zero is half

            // Keep track of edge
            edge_count++;    
          }
        }
        else if(edge_count == 1) {
          // Second edge of 0 bit: reset to first edge
          edge_count = 0;
        }
      } else {
        first_edge = false;
      }
    }
  }



  // Save time;
  last_edge_time = current_time;
}

// uint16_t average_sum;
// void continuous_cal(uint16_t new_value) {
//   average_sum = cal_value * CONTINUOUS_CAL_WEIGHT;
//   // // average_sum = average_sum * CONTINUOUS_; // Multiply by 16
//   average_sum += new_value;
//   cal_value = average_sum / (CONTINUOUS_CAL_WEIGHT + 1);
// }

// Read setup variables
volatile uint32_t read_setup_blocks;

// Communication variables with main loop
volatile uint32_t read_file_length;
volatile bool header_read;
volatile bool read_data_available; // 0 -> No data available, 1 -> Left part of the buffer available, 2 -> Right part of the buffer available

volatile int bit_counter;
volatile byte byte_read;
volatile int byte_counter;
volatile uint32_t bytes_read;
volatile uint32_t blocks_read;
void process_bit(bool bit_value) {
  if(read_phase == 2) {
    // Waiting for first start bit
    if(bit_value == 0) {

      bit_counter = 8;
      byte_counter = 0;
      bytes_read = 0;

      // Go to next read phase
      read_phase = 3; // Header read
    }
  }

  if(read_phase == 5) {
    // Waiting for first start bit
    if(bit_value == 0) {

      bytes_read = 0;
      byte_counter = 0;
      blocks_read = 0;
      

      // Go to next read phase
      read_phase = 6;
    }
  }

  if(read_phase == 3 || read_phase == 6) {
    // Actual bytes are read here

    // Detect framing errors
    if(bit_counter == 8 && bit_value != 0 || bit_counter == -1 && bit_value != 1) {
      // Framing error detected
      framing_error();
      return;
    }

    // Check if incoming bit is not start or end bit
    if(bit_counter >= 0 && bit_counter <= 7) {
      bitWrite(byte_read, bit_counter, bit_value);
    }

    if(--bit_counter < -1) {
      // If the end of byte

      // Process the read byte
      process_byte(byte_read);

      // Reset bit in byte
      bit_counter = 8;
    }

  }
}

void process_byte(byte byte_value) {
  
  if(read_phase == 3) {
    // Header read
    
    // If this is the last byte of the header
    if(bytes_read == HEADER_BUF_SIZE - 1) {
      // Reset bytes read
      bytes_read = 0;
      byte_counter = 0;
      blocks_read = 0;

      // Extract information from header
      setup_read_from_header();


      first_edge = true;
      cal_sum = 0;
      cal_count = 0;
      calibration_start = millis();
            
      // Switch to read phase 4
      read_phase = 4; // Internal calibration

      return;
    }

    // Save value in header buffer
    header_buf[bytes_read] = byte_value;

    bytes_read++;
  }

  if(read_phase == 6) {
    // Data read

    // Save value in data buffer
    data_buf[bytes_read] = byte_value;
    
    // If we're at the end of the data buffer
    if(bytes_read == DATA_BUF_SIZE - 1) {
      byte_counter = 0;
      // Reset bytes read
      bytes_read = 0;

      // Next block
      blocks_read++;

      // Notify main loop that there is data available
      read_data_available = true;

      // Check if we've read all blocks
      if(blocks_read >= read_setup_blocks) {
        // Read DONE!

        // Serial.println("Done!");

        // Stop reading further data
        read_phase = 0;
        return;
      }

      first_edge = true;
      cal_sum = 0;
      cal_count = 0;
      calibration_start = millis();

      // Go to recalibration
      read_phase = 4;

    } else {
      // Go to next byte
      bytes_read++;
    }
  }

}

void framing_error() {
  #ifdef DEBUG
  DebugSerial.println("Framing error! Aborting read...");
  #endif
  
  // Inform main loop
  read_aborted = true;

  // Actually stop read
  read_phase = 0;
}

void setup_read_from_header() {
  // Check that the tape identification string matches
  if(strncmp(header_buf, TAPE_IDENT_STR, TAPE_IDENT_STR_LEN) == 0) {

    // Copy file length to local variable
    memcpy(&read_file_length, header_buf + TAPE_IDENT_STR_LEN, 4);

    // Compute number of blocks
    if(read_file_length == 0)
      read_setup_blocks = 0;
    else
      read_setup_blocks = 1 + ((read_file_length - 1) / BLOCK_SIZE); // if x != 0

  } else {
    #ifdef DEBUG
    DebugSerial.println("Tape ident doesn't match!");
    #endif

    read_aborted = true;
    read_phase = 0;

    return;
  }

  // Inform main loop that we have the header
  header_read = true;
}

void start_read() {
  // Start read

  // Set calibration variables
  first_edge = true;
  cal_sum = 0;
  cal_count = 0;

  // Make main loop wait for calibration and header
  header_read = false;
  read_aborted = false;
  read_data_available = false;

  #ifdef DEBUG
  DebugSerial.println("Waiting for cal...");
  #endif

  // Set reader to initial calibration phase
  read_phase = 1; 
}
/*********************************************************************/

/************************   Write functions   ************************/
// Write setup variables
volatile uint32_t write_setup_blocks;

volatile bool bit_start;
volatile bool current_bit;
volatile int write_phase; // Write phase -> 0: not writing, 1: leader, 2: header, 3: recal leader, 4: data
volatile bool write_pin_state;

ISR(TIMER2_OVF_vect) {
  write_tick();
}

void write_tick() {
  #ifdef EXT_TRIG
  // Reset external trigger for oscilloscope
  digitalWrite(EXT_TRIGGER_PIN, LOW);
  #endif

  // Check if we are supposed to be still writing
  if (write_phase != 0) {
    // Check if first edge of bit
    if(bit_start) {
      // Get new bit
      current_bit = get_bit();
      
      #ifdef DEBUG
      DebugSerial.print(current_bit);
      #endif

      // On first edge, always invert
      toggle_write_pin();
    } else {
      if(!current_bit) {
        // Toggle in middle of bit only if bit is 0
        toggle_write_pin();
      }
    } 

    // We are now in a different position in the bit
    bit_start = !bit_start;
  } else {
    // When there isn't a write going on, ouptut LOW
    digitalWrite(DATA_WRITE_PIN, LOW);
  }
}

volatile int leader_counter;
volatile uint32_t blocks_written;
volatile uint32_t bytes_written;
volatile byte bit_in_byte;
volatile byte current_byte;
volatile bool request_fill;
volatile bool write_done;


void toggle_write_pin() {
  write_pin_state = !write_pin_state;
  digitalWrite(DATA_WRITE_PIN, write_pin_state);
}
 
bool get_bit() {
  
  // Check in which write phase we are
  if(write_phase == 1) {
    // We are in the leader write phase
    
    // Check if we are stil in the leader
    if(leader_counter < LEADER_LENGTH) {
      leader_counter++;
      return true; // Bits are all 1 in header
    } else {
      // Leader finished
      // Switch to header write phase
      
      // Set variables for header write
      bytes_written = 0;
      bit_in_byte = 0xFF;

      // Set write phase
      write_phase = 2;
    }
  }
  
  if (write_phase == 2) { 
    // We are in the header write phase

    // Check that we aren't outside the byte
    if(--bit_in_byte == 0xFE) {
      bit_in_byte = 8;
      
      // Need new byte

      // Check if there are still bytes to write
      if(bytes_written < HEADER_BUF_SIZE) {
        // Get current byte from header buffer
        current_byte = header_buf[bytes_written];
        // Next byte
        bytes_written++;

        #ifdef DEBUG
        DebugSerial.println();
        #endif

      } else {
        // Header finished
        // Switch to data write phase

        // Set variables for recal leader write
        blocks_written = 0;
        leader_counter = 0; // Set leader counter
        request_fill = true;

        // Set write phase
        write_phase = 3;
      }
    }

    // Check that we still are in write phase 2
    if(write_phase == 2) {
      // Check if this is the start bit
      if(bit_in_byte == 8) {

        #ifdef EXT_TRIG
        // Send external trigger pulse
        digitalWrite(EXT_TRIGGER_PIN, HIGH);
        #endif

        return false; // Start bit is always 0;
      } else if (bit_in_byte == 0xFF) {
        return true; // Stop bit is always 1;
      } else {
        // Get current bit from byte
        return bitRead(current_byte, bit_in_byte);
      }
    }
  }

  if(write_phase == 3) {
    // We are in the recal leader write phase
    
    // Check if we are stil in the leader
    if(leader_counter < RECAL_LEADER_LENGTH) {
      leader_counter++;
      return true; // Bits are all 1 in header
    } else {
      // Leader finished
      // Switch to header write phase
      
      // Set variables for data write
      bytes_written = 0;
      bit_in_byte = 0xFF;

      // Set write phase
      write_phase = 4;
    }
  }
  
  if (write_phase == 4) {
    // We are in the data write phase

    // Check that we aren't outside the byte
    if(--bit_in_byte == 0xFE) {
      bit_in_byte = 8;
      
      // Need new byte

      // Check if there are still bytes to write
      if(bytes_written >= DATA_BUF_SIZE) {
        // End of data buffer
        // Go back to beginning of buffer

        // Count block
        blocks_written++;

        // Go back to recal leader write
        leader_counter = 0; // Set leader counter
        request_fill = true;
        write_phase = 3;
        return 1; // Write 1
      }

      // Check if there are still blocks to write
      if(blocks_written >= write_setup_blocks) {
        // End write
        write_phase = 0;
        // Tell control loop that write is done
        write_done = true;
      }

      // Get current byte from data buffer
      current_byte = data_buf[bytes_written];
      // Next byte
      bytes_written++;

      #ifdef DEBUG
      DebugSerial.println();
      #endif
    }

    // Check that we are still in the data write phase
    if(write_phase == 4) {
      // Check if this is the start bit
      if(bit_in_byte == 8) {

        #ifdef EXT_TRIG
        // Send external trigger pulse
        digitalWrite(EXT_TRIGGER_PIN, HIGH);
        #endif

        return false; // Start bit is always 0;
      } else if (bit_in_byte == 0xFF) {
        return true; // Stop bit is always 1;
      } else {
        // Get current bit from byte
        return bitRead(current_byte, bit_in_byte);
      }
    }
  }
  
  return false;
}

void start_write() {
  // Check that we aren't writing already
  if(write_phase == 0) {
    
    // Set pin state
    bool write_pin_state = false;

    // Set leader counter
    leader_counter = 0;

    // Do not request data
    request_fill = false;

    // Set write done
    write_done = false;

    // Actually start the write
    write_phase = 1;
    bit_start = true;

  } else {
    Serial.println("Error starting write: write already in progress!");
  }  
}

// To be called when buffer fill times out
void abort_write() {
  // Communciate to PC that the write was aborted
  Serial.write(CMD_ABORT);

  #ifdef DEBUG
  DebugSerial.println("Write aborted!");
  #endif
  
  // Stop write
  write_phase = 0;
}

void setup_header_buf(uint32_t file_size) {
  // Clear buffer
  for(int i = 0; i < HEADER_BUF_SIZE; i++) {
    header_buf[i] = 0x00;
  }

  // Copy ident string
  memcpy(header_buf, TAPE_IDENT_STR, TAPE_IDENT_STR_LEN);

  // Copy length
  memcpy(header_buf + TAPE_IDENT_STR_LEN, &file_size, 4);
}
/*********************************************************************/

/****************** Serial communication functions *******************/
int update_block_in_buffer() {
  // Compute starting offset into buffer depending on which part is free
  // int offset = buffer_lock ? BLOCK_SIZE + CRC_SIZE : 0;
  int bytes_read = 0;
  unsigned long last_byte_time;

  // Tell interface program that we need new data
  Serial.write(CMD_DATA_REQUEST);
  
  // Read bytes from serial
  last_byte_time = millis();
  while(bytes_read < BLOCK_SIZE) {

    // Check for read timeout
    if(millis() - last_byte_time > BLOCK_READ_BYTE_TIMEOUT) {
      #ifdef DEBUG
      DebugSerial.println("Serial byte read timeout error!");
      #endif
      return 1;
      
    }

    if(Serial.available()) {
      data_buf[bytes_read] = Serial.read();
      bytes_read++;
      last_byte_time = millis();
    }
  }

  return 0;
}

int read_uint32_t_from_serial(uint32_t &number) {
  char bytes[4];  
  if(Serial.readBytes(bytes, 4) < 4) {
    return 1;
  }
  memcpy(&number, &bytes, 4);
  return 0;
}

void write_uint32_t_to_serial(uint32_t &number) {
  char bytes[4];
  memcpy(&bytes, &number, 4);
  Serial.write(bytes, 4);
  return 0;
}


/*********************************************************************/

/************************** CRC functions ****************************/
void calc_crc() {
  data_buf[512] = 1;
  data_buf[513] = 2;
}
/*********************************************************************/

/******************************** COMMANDS ********************************/
void handshake() {
  Serial.write("AT-1.0.0");

  #ifdef DEBUG
  DebugSerial.println("HANDSHAKE complete!");
  #endif
}

void file_read() {
  uint32_t file_length;
  uint32_t blocks_to_read;
  uint32_t blocks_read_local;
  int buffer_offset;
  
  // Begin listening for calibration leader
  start_read();

  // Wait for header read to be done
  while(!header_read && !read_aborted);

  if(read_aborted) {
    Serial.write(CMD_ABORT);
    return;
  }

  // Save file length
  file_length = read_file_length;
  blocks_to_read = read_setup_blocks;

  #ifdef DEBUG
  DebugSerial.println("Blocks to read: ");
  DebugSerial.println(blocks_to_read);
  #endif

  // Communicate file length to PC
  Serial.write(CMD_BEGIN);
  write_uint32_t_to_serial(file_length);

  // Main read loop
  blocks_read_local = 0;
  while(blocks_read_local < blocks_to_read) {
    
    // Wait for either data or read abort
    while(!read_data_available && !read_aborted);

    // If aborted, exit read
    if(read_aborted) {
      Serial.write(CMD_ABORT);
      return;
    }

    #ifdef DEBUG
    DebugSerial.println("rda: ");
    DebugSerial.println(read_data_available);
    #endif

    read_data_available = false;   

    // Send data over serial
    Serial.write(CMD_DATA_REQUEST); // Notify PC of data available
    Serial.write((byte *)data_buf, BLOCK_SIZE);

    blocks_read_local++; 
  }
}

void file_write() {
  uint32_t file_length;
  uint32_t blocks;
  uint32_t blocks_written;

  #ifdef DEBUG
  DebugSerial.println("Start file WRITE!");
  #endif

  // Read file length from serial
  if(read_uint32_t_from_serial(file_length) != 0) {

    #ifdef DEBUG
    DebugSerial.println("Exiting from file write");
    #endif

    return;
  }

  #ifdef DEBUG
  DebugSerial.print("File length: ");
  DebugSerial.println(file_length);
  #endif

  // Compute file size in blocks
  if(file_length == 0)
    blocks = 0;
  else
    blocks = 1 + ((file_length - 1) / BLOCK_SIZE); // if x != 0

  #ifdef DEBUG
  DebugSerial.print("Blocks: ");
  DebugSerial.println(blocks);
  #endif

  delay(500);

  #ifdef DEBUG
  DebugSerial.println("Starting write...");
  #endif

  // Populate header with correct data
  setup_header_buf(file_length);

  // Get first block into the buffer
  // if(update_block_in_buffer() != 0) { 
  //   abort_write();
  //   return;    
  // } 

  // calc_crc();

  // Start write
  write_setup_blocks = blocks;
  start_write();

  // Start checking if new data is needed
  for(int i = 0; i < blocks; i++) {
    // Wait for data to be requested by write routine
    while(!request_fill);

    // Fill block with new data
    if(update_block_in_buffer() != 0) { 
      abort_write();
      return;
    } 
    calc_crc();

    request_fill = false;
  }

  while(!write_done);

  // Alert PC of write done
  Serial.write(CMD_SUCCESS);

  #ifdef DEBUG
  DebugSerial.println("Write done!");
  #endif
}
/**************************************************************************/

void setup() {
  // Setup serial communication
  Serial.begin(MAIN_SERIAL_SPEED);

  #ifdef DEBUG 
  DebugSerial.begin(DEBUG_SERIAL_SPEED);
  #endif
  

  // Setup pins
  pinMode(DATA_WRITE_PIN, OUTPUT);
  pinMode(DATA_READ_PIN, INPUT);
  #ifdef EXT_TRIG
  pinMode(EXT_TRIGGER_PIN, OUTPUT);
  #endif

  // Setup write interrupt (trigger roughly @ 2kHz)
  TCCR2A = 0b00000000; // Set clear on OCRB, set on BOTTOM
  TCCR2B = 0b00000011; // Set clock from prescaler(/8)
  TIMSK2 = 0b00000001; // Enable interrupt on overflow
  TCNT2 = 0;           // Reset counter register

  // Setup read edge change interrupt
  attachInterrupt(digitalPinToInterrupt(DATA_READ_PIN), read_edge_isr, CHANGE);

  #ifdef DEBUG
  DebugSerial.println("READY!");
  #endif
}

void loop() {
  if(Serial.available() > 0) {

    switch(Serial.read()) {
      case CMD_HANDSHAKE:
        handshake();
        break;
      case CMD_READ:
        file_read();
        break;
      case CMD_WRITE:
        file_write();
        break;
    }
  }
}