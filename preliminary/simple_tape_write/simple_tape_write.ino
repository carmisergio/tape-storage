void setup() {
  // put your setup code here, to run once:

  pinMode(11, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  TCCR2A = 0b10000011; // Set clear on OCRB, set on BOTTOM
  //TCCR2A = 0b00000000; // Set clear on OCRB, set on BOTTOM
                       // Waveform Generation -> Fast PWM
  TCCR2B = 0b0000111; // Set clock from prescaler(256)
  TCNT2 = 0;           // Reset counter register
  OCR2A = 0xFF;
  OCR2B = 0;         // Pin turn off point
  TIMSK2 = 0b00000001; // Enable overflow interrupt

}

volatile byte data[10] = {0b00000000, 0b11111111, 0b00000000, 0b11111111, 0b10101010, 0b01010101, 0x11001100, 0x11110000};
volatile byte 
volatile unsigned int byte_ptr = 0;
volatile unsigned int bit_ptr = 7;
volatile bool done = false;

ISR(TIMER2_OVF_vect) {
  if(!done) {
    byte bit = bitRead(data[byte_ptr], bit_ptr); // Get bit from byte
      //  Serial.println(bit_ptr);
      if(bit == 1) {
        OCR2A = 192;
      } else {
        OCR2A = 192;
      }

      // Increment and reset pointers correctly
      if(bit_ptr == 0) {
        bit_ptr = 8;
        done = true;
      }
      bit_ptr--;
  } else {
    TCCR2A = 0b10000000;
    TCCR2B = 0b00000000;
    TIMSK2 = 0b00000000;
    digitalWrite(11, LOW);
  }
  
}

void loop() {
}
