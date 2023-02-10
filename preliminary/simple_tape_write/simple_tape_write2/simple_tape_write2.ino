#define DATA_BUFFER_LENGTH 512
#define DATA_WRITE_PIN 11
#define DATA_READ_PIN 3


void setup() {
  // put your setup code here, to run once:

  pinMode(DATA_WRITE_PIN, OUTPUT);
  pinMode(DATA_READ_PIN, INPUT);

  // Setup timer 2 to trigger interrupt at roughly 2kHz
  TCCR2A = 0b00000000; // Set clear on OCRB, set on BOTTOM
  TCCR2B = 0b00000010; // Set clock from prescaler(256)
  TCNT2 = 0;           // Reset counter register
  TIMSK2 = 0b00000001; // Enable overflow interrupt

  Serial.begin(38400);

  // Put some data in the read buffer
  fill_data(0b11110101);

  // Setup data read interrupts
  attachInterrupt(digitalPinToInterrupt(DATA_READ_PIN), data_read_rising, RISING);
  attachInterrupt(digitalPinToInterrupt(DATA_READ_PIN), data_read_falling, FALLING);
}


//volatile bool data_bits[48] = {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,1,0,0,1,1,1,1,0,0,0,0};
//volatile int bits_to_write = 0;
//volatile byte write_phase = 0;
//volatile bool bit;

// Data buffer
volatile byte data_buffer[DATA_BUFFER_LENGTH];

// Read vars
volatile long unsigned int bit_start;
volatile long unsigned int bit_fall;
volatile long unsigned int bit_end;
volatile unsigned short bits_to_read;
volatile bool first_bit;

// Write vars
volatile unsigned short bytes_to_write;
volatile unsigned short byte_to_write_index;
volatile byte byte_to_write;
volatile byte bit_in_byte;
volatile byte write_phase;
volatile byte write_end_counter;
volatile bool write_bit_value;

// Serial communication
char serial_in;

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
        byte_to_write_index++;
        byte_to_write = data_buffer[byte_to_write_index];
      }      
    }
    // Handle generating end pulse
  } else if (write_end_counter > 0) {
    digitalWrite(DATA_WRITE_PIN, write_end_counter != 1);
    write_end_counter--;  
  }
}

// Data Read interrupts

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


/*
 * Fills data buffer with specific value
 * Parameters:
 *  - byte value: value to fill the data buffer with
 */
void fill_data(byte value) {
  for(int i = 0; i < 512; i++) {
    data_buffer[i] = value;
  }
  data_buffer[0] = 0b10000000;
  data_buffer[1] = 0b10000000;
  data_buffer[2] = 0b10000000;
  data_buffer[3] = 0b10000000;
  data_buffer[4] = 0b11111111;
  data_buffer[5] = 0b00000000;
  data_buffer[6] = 0b10101010;
  data_buffer[7] = 0b01010101;
  data_buffer[8] = 0b11110000;
  data_buffer[9] = 0b00001111;
}

void start_read(unsigned short length) {
  first_bit = true;
  bits_to_read = length;
}

void start_write(unsigned short length) {
  bytes_to_write = length;
  byte_to_write_index = 0;
  bit_in_byte = 7;
  write_phase = 0;
  write_end_counter = 10;
  byte_to_write = data_buffer[0];
}

void loop() {
  if(Serial.available() > 0) {
    serial_in = Serial.read();

    switch(serial_in) {
      case 'r':
        Serial.println("Starging READ...");
        start_read(10);
        break;
      
      case 'w':
        Serial.println("Starging WRITE...");
        start_write(10);
    }
  }
}
