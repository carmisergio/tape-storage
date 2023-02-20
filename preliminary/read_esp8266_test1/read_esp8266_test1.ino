
// Read vars
volatile long unsigned int bit_start;
volatile long unsigned int bit_fall;
volatile long unsigned int bit_end;
volatile unsigned short bits_to_read;
volatile bool first_bit;

// Serial communication
char serial_in;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  pinMode(5, INPUT);
  attachInterrupt(digitalPinToInterrupt(5), data_read_ISR, CHANGE);

}

// Data Read interrupts

volatile char byte_read;
volatile byte bit_in_byte;
volatile byte bytes_read;

ICACHE_RAM_ATTR void data_read_ISR() {
  if(digitalRead(5)) {
    if(bits_to_read > 0) {
      if(first_bit) {
        //Serial.println("First bit!");
        //Serial.println();
        bit_start = micros();
        first_bit = false;
      } else {
        bit_end = micros();
        if(bit_fall - bit_start > bit_end - bit_fall) {
          bitSet(byte_read, bit_in_byte);
        } else {
          bitClear(byte_read, bit_in_byte);
        }
        bits_to_read--;
        
        bit_in_byte--;
        if(bit_in_byte == 0xFF) {
          
          if(byte_read < 10)
            Serial.print("0");
          
          Serial.print(byte_read, HEX);
          Serial.print(" ");

          bytes_read++;
          if(bytes_read > 32) {
            Serial.println();
            bytes_read = 0;
          }

          bit_in_byte = 7;
        }

        bit_start = bit_end;
      }
    }
  } else {
    bit_fall = micros();
  }
  
}

void start_read(unsigned short length) {
  first_bit = true;
  bits_to_read = length;
  bit_in_byte = 7;
  bytes_read = 0;
}

void loop() {
  if(Serial.available() > 0) {
    serial_in = Serial.read();

    switch(serial_in) {
      case 'r':
        Serial.println("Starging READ...");
        start_read(Serial.parseInt());
        break;
    }
  }
}
