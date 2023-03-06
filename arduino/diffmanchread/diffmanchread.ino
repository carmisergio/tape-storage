#define DEBUG

// Pin configuration
#define DATA_READ_PIN 3

// Buffer sizes
#define BLOCK_SIZE 10 // TODO change to 512
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
// const char TAPE_IDENT_STR[TAPE_IDENT_STR_LEN]  = "VCS01SER";

// Data encoding defines
#define LEADER_LENGTH 10 // Amount of 1 bits that are sent as leader // TODO change back to 500

// Data buffers
volatile byte header_buf[HEADER_BUF_SIZE];
volatile byte data_buf[DATA_BUF_SIZE]; 


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

      Serial.println();

      bit_counter = -1;

      // Go to next read phase
      read_phase = 3;
    } else {
      Serial.print(".");
    }
  }

  if(read_phase == 3) {
    if(++bit_counter > 9) {
      bit_counter = 0;
      Serial.println();
    }

    Serial.print(bit_value);

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

  Serial.println("Waiting for cal...");

  // Set reader to initial calibration phase
  read_phase = 1; 
}
/*********************************************************************/

/****************** Serial communication functions *******************/
/*********************************************************************/

/************************** CRC functions ****************************/
/*********************************************************************/

void test_read() {
  start_read();
}

void setup() {
  // Setup serial communication
  Serial.begin(115200);
  Serial.println();

  // Setup pins
  pinMode(DATA_READ_PIN, INPUT);

  // Setup read edge change interrupt
  attachInterrupt(digitalPinToInterrupt(DATA_READ_PIN), read_edge_isr, CHANGE);

  // Setup timeout check interrupt (trigger roughly @ 2kHz)
  // TCCR2A = 0b00000000; // Set clear on OCRB, set on BOTTOM
  // TCCR2B = 0b00000011; // Set clock from prescaler(/8)
  // TIMSK2 = 0b00000001; // Enable interrupt on overflow
  // TCNT2 = 0;           // Reset counter register

  delay(1000);

  test_read();
}

void loop() {
}

// First run calibration read phase 1
// Require at least N edges of time within certain variation % (maybe 15) before considering initial calibration complete
// Then start forwarding bytes to store_bit(bool bit_value) read phase 2
// It will wait until it sees a 0 bit to switch to read_phase 3

// While reading data adjust calibration by weighted average with last bit time and calibration value
// If time of bit read is outside of spec (> 2x or < 1/4 calibration value) -> Read error

// Implement read timeout on actual arduino hardware