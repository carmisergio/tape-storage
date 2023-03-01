#define DEBUG

// Pin configuration
#define DATA_WRITE_PIN 11
#define EXT_TRIGGER_PIN 12

// Buffer sizes
#define BLOCK_SIZE 10 // TODO change to 512
#define HEADER_SIZE 2 // TODO change to 32
#define CRC_SIZE 2
#define DATA_BUF_SIZE (BLOCK_SIZE + CRC_SIZE) * 2
#define HEADER_BUF_SIZE HEADER_SIZE + CRC_SIZE

// Data encoding defines
#define LEADER_LENGTH 10 // Amount of 1 bits that are sent as leader // TODO change back to 500

// Data buffers
volatile byte header_buf[HEADER_BUF_SIZE] = {0b11111111, 0b10101010, 1, 1}; // TODO remove literal
volatile byte data_buf[DATA_BUF_SIZE] = {0b00000000, 0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000, 0b11111111, 0b00000000, 0b00000000,
                                         0b01111111, 0b10101010, 0b01010101, 0b11110000, 0b00001111, 0b11001100, 0b00110011, 0b10110011, 0b00000001, 0b00010000, 0b00000000, 0b00000000,}; 

// TODO remove literal

// Write setup variables
volatile uint32_t write_setup_blocks;

/************************   Write functions   ************************/
volatile bool bit_start;
volatile bool current_bit;
volatile int write_phase; // Write phase -> 0: not writing, 1: leader, 2: header, 3: data

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
      bit_in_byte = 0;

      // Set write phase
      write_phase = 2;
    }
  }
  
  if (write_phase == 2) { 
    // We are in the header write phase

    // Check that we aren't outside the byte
    if(--bit_in_byte == 0xFF) {
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
        bit_in_byte = 0;

        // Set write phase
        write_phase = 3;
      }
    }

    // Check that we still are in write phase 2
    if(write_phase == 2) {
      // Check if this is the start bit
      if(bit_in_byte == 8) {
        return false; // Start bit is always 0;
      } else {
        // Get current bit from byte
        return bitRead(current_byte, bit_in_byte);
      }
    }
  }
  
  if (write_phase == 3) {
    // We are in the data write phase

    // Check that we aren't outside the byte
    if(--bit_in_byte == 0xFF) {
      bit_in_byte = 8;
      
      // Need new byte

      // Check if we are at the middle of the buffer (going into the next block)
      if(bytes_written == BLOCK_SIZE + CRC_SIZE) {
        // Count block
        blocks_written++;
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
      }

      // Check if there are still blocks to write
      if(blocks_written >= write_setup_blocks) {
        // End write
        write_phase = 0;
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
      } else {
        // Get current bit from byte
        return bitRead(current_byte, bit_in_byte);
      }
    }
  }
  
  return false;
}

void toggle_write_pin() {
  digitalWrite(DATA_WRITE_PIN, !digitalRead(DATA_WRITE_PIN));
}

void start_write() {
  // Check that we aren't writing already
  if(write_phase == 0) {
    
    // Set leader counter
    leader_counter = 0;

    // Actually start the write
    write_phase = 1;
    bit_start = true;

  } else {
    Serial.println("Error starting write: write already in progress!");
  }  
}
/*********************************************************************/

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

  write_setup_blocks = 3;
  start_write();

  delay(17000);
  Serial.println();
  Serial.println();

  write_setup_blocks = 1;
  start_write();

  delay(10000);
  Serial.println();
  Serial.println();

  write_setup_blocks = 2;
  start_write();
}

void loop() {
}