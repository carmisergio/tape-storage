#include <SoftwareSerial.h>


// Feature enables
#define DEBUG
#define EXT_TRIG

// Pin configuration
#define DATA_WRITE_PIN 4
#define DATA_READ_PIN 3
#define EXT_TRIGGER_PIN 12

// Serial configuration
#define MAIN_SERIAL_SPEED 9600
#define DEBUG_SERIAL_SPEED 115200
#define DEBUG_SERIAL_RX 12
#define DEBUG_SERIAL_TX 11

// Buffer sizes
#define BLOCK_SIZE 512
#define HEADER_SIZE 32
#define CRC_SIZE 2
#define DATA_BUF_SIZE (BLOCK_SIZE + CRC_SIZE) * 2
#define HEADER_BUF_SIZE HEADER_SIZE + CRC_SIZE

// Calibration config
#define CAL_SUCCESSFUL_PULSES 20
#define CAL_ACCEPTED_VARIATION 0.15 // 15% accepted variation in calibration timing
#define CAL_VALID_MAX 2000 // uS
#define CAL_VALID_MIN 500 // uS

// Tape identification string
#define TAPE_IDENT_STR_LEN 8
const char TAPE_IDENT_STR[]  = "VCS01SER";

// Data encoding defines
#define LEADER_LENGTH 2000

// Data buffers
volatile byte header_buf[HEADER_BUF_SIZE];
volatile byte data_buf[DATA_BUF_SIZE]; 

// Software serial for debug
#ifdef DEBUG
SoftwareSerial DebugSerial(DEBUG_SERIAL_RX, DEBUG_SERIAL_TX);
#endif

/************************   Read functions   ************************/
volatile int read_phase; // Read phase -> 0: not reading, 1: initial calibration, 2: leader wait, 3: header, 4: data
volatile bool first_edge;
volatile uint32_t current_time;
volatile uint32_t last_edge_time;
volatile uint16_t current_time_delta;
volatile uint32_t cal_sum;
volatile int cal_count;
volatile int cal_value;
volatile byte edge_count;

// Data read edge change interrupt
// ICACHE_RAM_ATTR 
void read_edge_isr() {

  // Get time of current edge
  current_time = micros();

  // Calculate time delta of current transition
  current_time_delta = current_time - last_edge_time;

  // See what phase we're in
  if(read_phase == 1) {
    // Calibration phase
    if(!first_edge) {

      Serial.println(current_time_delta);

      if(current_time_delta <= CAL_VALID_MAX && current_time_delta >= CAL_VALID_MIN) {
        // Serial.print(".");
        // Serial.println(current_time_delta);

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

          Serial.println();
          Serial.print("Cal success! Value: ");
          Serial.println(cal_value);
          Serial.println("Waiting for first start bit...");
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
  }

  if(read_phase > 1) {

    if(!first_edge) {
      // If we're on at least the second edge
      if(edge_count == 0) {

        // Check time with calibrated threashold value
        if(current_time_delta > ((cal_value * 3) / 4)) {

          // Received bit 1
          process_bit(1);
        } else {

          // Received bit 0
          process_bit(0);  

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

  // Save time;
  last_edge_time = current_time;
}

volatile int bit_counter;
void process_bit(bool bit_value) {
  if(read_phase == 2) {
    // Waiting for first start bit
    if(bit_value == 0) {

      DebugSerial.println();

      bit_counter = -1;

      // Go to next read phase
      read_phase = 3;
    } else {
      DebugSerial.print(".");
    }
  }

  if(read_phase == 3) {
    if(++bit_counter > 9) {
      bit_counter = 0;
      DebugSerial.println();
    }

    DebugSerial.print(bit_value);

  }
}

// Timeout check interrupt
// ISR(TIMER2_OVF_vect) {
// }

void start_read() {
  // Start read

  // Set calibration variables
  first_edge = true;
  cal_sum = 0;
  cal_count = 0;

  DebugSerial.println("Waiting for cal...");

  // Set reader to initial calibration phase
  read_phase = 1; 
}
/*********************************************************************/

/************************   Write functions   ************************/
// Write setup variables
volatile uint32_t write_setup_blocks;

volatile bool bit_start;
volatile bool current_bit;
volatile int write_phase; // Write phase -> 0: not writing, 1: leader, 2: header, 3: data
volatile bool write_pin_state;

ISR(TIMER2_OVF_vect) {
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
volatile int buffer_lock; // true -> left part locked, false -> right part locked
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

        // Set variables for data write
        blocks_written = 0;
        bytes_written = 0;
        bit_in_byte = 0xFF;
        buffer_lock = true; // Lock left part of buffer
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
  
  if (write_phase == 3) {
    // We are in the data write phase

    // Check that we aren't outside the byte
    if(--bit_in_byte == 0xFE) {
      bit_in_byte = 8;
      
      // Need new byte

      // Check if we are at the middle of the buffer (going into the next block)
      if(bytes_written == BLOCK_SIZE + CRC_SIZE) {
        if(request_fill) {
          // Data was not correctly filled by serial
          abort_write();
        }

        // Count block
        blocks_written++;
        
        // Request fill of left part of buffer
        buffer_lock = false;
        request_fill = true;
      }

      // Check if there are still bytes to write
      if(bytes_written >= DATA_BUF_SIZE) {
        // End of data buffer
        // Go back to beginning of buffer

        if(request_fill) {
          // Data was not correctly filled by serial
          abort_write();
        }

        // Reset variables
        bytes_written = 0;
        bit_in_byte = 8;

        // Count block
        blocks_written++;

        // Request fill of left part of buffer
        buffer_lock = true;
        request_fill = true;
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
    if(write_phase == 3) {
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

    // Lock right part of buffer (so first fill from serial will fill left part)
    buffer_lock = false;

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
  DebugSerial.println("Write aborted!");
  
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
volatile byte current_value;
void update_block_in_buffer() {
  // Compute starting offset into buffer depending on which part is free
  int offset = buffer_lock ? BLOCK_SIZE + CRC_SIZE : 0; 

  for(int i = 0; i < BLOCK_SIZE; i++) {
    data_buf[offset + i] = current_value;
  }

  current_value++;



}

int read_uint32_t_from_serial(uint32_t &number) {
  char bytes[4];  
  if(Serial.readBytes(bytes, 4) < 4) {
    return -1;
  }
  memcpy(&number, &bytes, 4);
  return 0;
}

/*********************************************************************/

/************************** CRC functions ****************************/
void calc_crc() {
  // Compute starting offset into buffer depending on which part is free
  int offset = buffer_lock ? 2 * BLOCK_SIZE + CRC_SIZE : BLOCK_SIZE; 
  data_buf[offset] = 1;
  data_buf[offset + 1] = 2;
}
/*********************************************************************/

void test_read() {
  start_read();
}

/******************************** COMMANDS ********************************/
void handshake() {
  Serial.write("AT-1.0.0");
  DebugSerial.println("HANDSHAKE complete!");
}

void file_write() {
  uint32_t file_length;
  uint32_t blocks;
  uint32_t blocks_written;

  DebugSerial.println("Start file WRITE!");

  // Read file length from serial
  if(read_uint32_t_from_serial(file_length) != 0) {
    DebugSerial.println("Exiting from file write");
    return;
  }

  DebugSerial.print("File length: ");
  DebugSerial.println(file_length);

  // Compute file size in blocks
  if(file_length == 0)
    blocks = 0;
  else
    blocks = 1 + ((file_length - 1) / BLOCK_SIZE); // if x != 0

  DebugSerial.print("Blocks: ");
  DebugSerial.println(blocks);

  delay(500);
  DebugSerial.println("Starting write...");

  // Populate header with correct data
  setup_header_buf(file_length);

  // Get first block into the buffer
  update_block_in_buffer(); 
  calc_crc();

  // Start write
  write_setup_blocks = blocks;
  start_write();

  // Start checking if new data is needed
  for(int i = 1; i < blocks; i++) {
    // Wait for data to be requested by write routine
    while(!request_fill);

    // Fill block with new data
    update_block_in_buffer();
    calc_crc();

    request_fill = false;
  }

  while(!write_done);

  Serial.println("Write done!");
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

  DebugSerial.println("READY!");
}

void loop() {
  if(Serial.available() > 0) {

    switch(Serial.read()) {
      case 'H':
        handshake();
        break;
      case 'R':
        test_read();
        break;
      case 'W':
        file_write();
        break;
    }
  }
}