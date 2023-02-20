#include <SoftwareSerial.h>

// Defines
#define DEBUG_TX_PIN 7
#define DEBUG_RX_PIN 8

// Commands
#define CMD_HANDSHAKE 'H'
#define CMD_READ 'R'
#define CMD_WRITE 'W'

#define BLOCK_SIZE 512
#define HEADER_LEN 32
#define RW_BUF_LEN BLOCK_SIZE + 2 // 512 for data + 2 for CRC
#define HEADER_BUF_LEN HEADER_LEN + 2
#define DATA_WRITE_PIN 11
#define DATA_READ_PIN 3

#define TAPE_IDENT_STR_LEN 8
const char TAPE_IDENT_STR[TAPE_IDENT_STR_LEN]  = "VCS01SER";

// Data buffers
volatile byte header_buf[HEADER_BUF_LEN];
volatile byte rw_buf_1[RW_BUF_LEN];
volatile byte rw_buf_2[RW_BUF_LEN];

// Read vars
volatile long unsigned int bit_start;
volatile long unsigned int bit_fall;
volatile long unsigned int bit_end;
volatile unsigned short bits_to_read;
volatile bool first_bit;

// Write vars
volatile bool write_header;
volatile bool write_buf_1;
volatile uint32_t blocks_to_write;
volatile unsigned short bytes_to_write = 0;
volatile unsigned short byte_to_write_index;
volatile byte byte_to_write;
volatile byte bit_in_byte;
volatile byte write_phase;
volatile byte write_end_counter;
volatile bool write_bit_value;

// Serial vars
char serial_in;

SoftwareSerial debugSerial(DEBUG_RX_PIN, DEBUG_TX_PIN);

/************************ READ AND WRITE INTERUPTS ************************/
// Write interrupt
ISR(TIMER2_OVF_vect) {
  
  if(bytes_to_write > 0) {

    // Get bit that has to be written
    write_bit_value = bitRead(byte_to_write, bit_in_byte);

    if(write_phase == 0) {
      digitalWrite(DATA_WRITE_PIN, HIGH);
    } else if (write_phase == 1 && !write_bit_value) {
      digitalWrite(DATA_WRITE_PIN, LOW); // DigitalWrite will happen earlier if bit is 0
    } else if (write_phase == 3){
      digitalWrite(DATA_WRITE_PIN, LOW);
    }

    // Go to next write phase
    write_phase++;

    // We are at the end of a write cycle
    if(write_phase > 3) {

      // Go to start of bit write cycle
      write_phase = 0;
      
      // Next bit in the byte
      bit_in_byte--;

      // Check if this was the last bit in the byte
      // Overflow (FF) means the byte was decremented below 0
      if(bit_in_byte == 0xFF) {
        
        // Go back to first bit
        bit_in_byte = 7;
        
        // Next byte
        bytes_to_write--;
        if(bytes_to_write == 0) {
          next_buffer();
        } else {
          byte_to_write_index++;
          // Get value of next byte
          if(write_header) {
            byte_to_write = header_buf[byte_to_write_index];
          } else if (write_buf_1) {
            byte_to_write = rw_buf_1[byte_to_write_index];
          } else {
            byte_to_write = rw_buf_2[byte_to_write_index];
          }
        }
      }      
    }
    // Handle generating end pulse
  } else if (write_end_counter > 0) {
    digitalWrite(DATA_WRITE_PIN, write_end_counter != 1);
    write_end_counter--;  
  }
}

void next_buffer() {
  if(blocks_to_write > 0) {
    write_buf_1 = !write_buf_1;
    write_header = false;
    bytes_to_write = RW_BUF_LEN;
    byte_to_write_index = 0;
    bit_in_byte = 7;
    write_phase = 0;
    write_end_counter = 10;
    blocks_to_write--;
    if(write_buf_1) {
      byte_to_write = rw_buf_1[0];
    } else {
      byte_to_write = rw_buf_2[0];
    }
  }
}

// Read interrupts
void data_read_rising() {
  Serial.println("RISING!");
  if(bits_to_read > 0) {
    if(first_bit) {
      Serial.println("First bit!");
      Serial.println();
      bit_start = micros();
      first_bit = false;
    } else {
      bit_end = micros();
      if(bit_fall - bit_start > bit_end - bit_fall) {
        Serial.print("1");
      } else {
        Serial.print("0");
      }
      bits_to_read--;
    }
  }
}

void data_read_falling() {
  bit_fall = micros();
}
/**************************************************************************/

void start_read(unsigned short length) {
  first_bit = true;
  bits_to_read = length;
}

void start_data_write() {
  // Setup to write header
  write_header = true;
  bytes_to_write = HEADER_BUF_LEN;
  byte_to_write_index = 0;
  bit_in_byte = 7;
  write_phase = 0;
  write_end_counter = 10;
  byte_to_write = header_buf[0];
}

int read_uint32_t_from_serial(uint32_t &number) {
  char bytes[4];  
  //debugSerial.println("Reading uint32_t from serial");  
  if(Serial.readBytes(bytes, 4) < 4) {
    return -1;
  }
  debugSerial.println((int)bytes[0]);
  memcpy(&number, &bytes, 4);
  return 0;
}

void set_header(uint32_t &file_length) {

  debugSerial.println(file_length);

  // Copy identification string into header buffer
  memcpy(header_buf, TAPE_IDENT_STR, TAPE_IDENT_STR_LEN);

  // Copy file length into header buffer
  memcpy(header_buf + TAPE_IDENT_STR_LEN, &file_length, 4);

  for(int i = 0; i < HEADER_BUF_LEN; i++) {
    debugSerial.print(header_buf[i], HEX);
  }
  debugSerial.println();
}


/******************************** COMMANDS ********************************/
void handshake() {
  Serial.println("AT-1.0.0");
  debugSerial.println("HANDSHAKE complete!");
}

////////////////////////////////////////////////////////////////////////////// TODO SOMETHING DOESN'T WORK AND IT DOESN'T READ THE LENGTH ANYMORE, BUT DOES IF I SEND IT LATER
////// MUST BE CLEARING SERIAL BUFFER BETWEEN READ AND READBYTES
void file_write() {
  uint32_t file_length;

  //debugSerial.println("Start file WRITE!");

  // Read file length from serial
  if(read_uint32_t_from_serial(file_length) != 0) {
    debugSerial.println("Exiting from file write");
    return;
  }

  debugSerial.print("File length: ");
  debugSerial.println(file_length);

  // Compute file size in blocks
  if(file_length == 0)
    blocks_to_write = 0;
  else
    blocks_to_write = 1 + ((file_length - 1) / BLOCK_SIZE); // if x != 0

  debugSerial.print("Blocks: ");
  debugSerial.println(blocks_to_write);

  delay(500);
  debugSerial.println("Starting write...");

  // Populate header with correct data
  set_header(file_length);

  for(int i = 0; i < 512; i++) {
    rw_buf_1[i] = 1;
    rw_buf_2[i] = 2;
  }

  start_data_write();
  
}
/**************************************************************************/

void setup() {
  debugSerial.begin(9600);

  Serial.begin(38400);

  pinMode(DATA_WRITE_PIN, OUTPUT);

  // Setup timer 2 to trigger interrupt at roughly 2kHz
  TCCR2A = 0b00000000; // Set clear on OCRB, set on BOTTOM
  TCCR2B = 0b00000010; // Set clock from prescaler(256)
  TCNT2 = 0;           // Reset counter register
  TIMSK2 = 0b00000001; // Enable overflow interrupt
  // Setup data read interrupts
  attachInterrupt(digitalPinToInterrupt(DATA_READ_PIN), data_read_rising, RISING);
  attachInterrupt(digitalPinToInterrupt(DATA_READ_PIN), data_read_falling, FALLING);
}

void loop() {
  if(Serial.available() > 0) {

    // Read serial character
    serial_in = Serial.read();

    switch(serial_in) {
      case 'H':
        handshake();
        break;
      case 'W':
        file_write();
        break;
    }
  }
}
