#define DEBUG

// Pin configuration
#define DATA_WRITE_PIN 4
#define EXT_TRIGGER_PIN 12

// Buffer sizes
#define BLOCK_SIZE 512 // TODO change to 512
#define HEADER_SIZE 32
#define CRC_SIZE 2
#define DATA_BUF_SIZE (BLOCK_SIZE + CRC_SIZE) * 2
#define HEADER_BUF_SIZE HEADER_SIZE + CRC_SIZE

// Tape identification string
#define TAPE_IDENT_STR_LEN 8
const char TAPE_IDENT_STR[]  = "VCS01SER";

// Data encoding defines
#define LEADER_LENGTH 2000 // Amount of 1 bits that are sent as leader // TODO change back to 500

// Data buffers
volatile byte header_buf[HEADER_BUF_SIZE];
volatile byte data_buf[DATA_BUF_SIZE]; 

// TODO remove literal

// Write setup variables
volatile uint32_t write_setup_blocks;

/************************   Write functions   ************************/
volatile bool bit_start;
volatile bool current_bit;
volatile int write_phase; // Write phase -> 0: not writing, 1: leader, 2: header, 3: data
volatile bool write_pin_state;

ISR(TIMER2_OVF_vect) {
  // Reset external trigger for oscilloscope
  digitalWrite(EXT_TRIGGER_PIN, LOW);

  // Check if we are supposed to be still writing
  if (write_phase != 0) {
    // Check if first edge of bit
    if(bit_start) {
      // Get new bit
      current_bit = get_bit();
      
      #ifdef DEBUG
      Serial.print(current_bit);
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
        Serial.println();
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
      Serial.println();
      #endif
    }

    // Check that we are still in the data write phase
    if(write_phase == 3) {
      // Check if this is the start bit
      if(bit_in_byte == 8) {
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
  Serial.println("Write aborted!");
  
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
/*********************************************************************/

/************************** CRC functions ****************************/
void calc_crc() {
  // Compute starting offset into buffer depending on which part is free
  int offset = buffer_lock ? 2 * BLOCK_SIZE + CRC_SIZE : BLOCK_SIZE; 
  data_buf[offset] = 1;
  data_buf[offset + 1] = 2;
}
/*********************************************************************/

void test_write() {
  int blocks = 20;
  int block_counter;

  setup_header_buf(10240);
  
  current_value = 1;
  update_block_in_buffer(); // Get first block into the buffer
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

void setup() {
  // Setup serial communication
  Serial.begin(115200);

  // Setup pins
  pinMode(DATA_WRITE_PIN, OUTPUT);
  pinMode(EXT_TRIGGER_PIN, OUTPUT);

  // Setup write interrupt (trigger roughly @ 2kHz)
  TCCR2A = 0b00000000; // Set clear on OCRB, set on BOTTOM
  TCCR2B = 0b00000011; // Set clock from prescaler(/8)
  TIMSK2 = 0b00000001; // Enable interrupt on overflow
  TCNT2 = 0;           // Reset counter register

  delay(1000);

  test_write();
}

void loop() {
}