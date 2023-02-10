
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
          Serial.print("1");
        } else {
          Serial.print("0");
        }
        bits_to_read--;
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
}

void loop() {
  if(Serial.available() > 0) {
    serial_in = Serial.read();

    switch(serial_in) {
      case 'r':
        Serial.println("Starging READ...");
        start_read(80);
        break;
    }
  }
}
