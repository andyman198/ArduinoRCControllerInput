
/*
  Arduino RC Controller Input
  ---------------------------
  Lesson 1: Read RC Controller Input and outputs the values to the plotter

  Read more about this example on the blog post at https://www.andyvickers.net/

  Author: Andy Vickers
  Downloaded From: https://github.com/andyman198/ArduinoRCControllerInput
  
*/

// Set the port speed for host communication
#define SERIAL_PORT_SPEED 115200

// Set the size of the arrays (increase for more channels)
#define RC_NUM_CHANNELS 4

// Set up our receiver channels - these are the channels from the receiver
#define RC_CH1  0 // Right Stick LR
#define RC_CH2  1 // Right Stick UD
#define RC_CH3  2 // Left  Stick UD
#define RC_CH4  3 // Left  Stick LR

// Set up our channel pins - these are the pins that we connect to the receiver
#define RC_CH1_INPUT  18 // receiver pin 1
#define RC_CH2_INPUT  19 // receiver pin 2
#define RC_CH3_INPUT  20 // receiver pin 3
#define RC_CH4_INPUT  21 // receiver pin 4

// Set up some arrays to store our pulse starts and widths
uint16_t RC_VALUES[RC_NUM_CHANNELS];
uint32_t RC_START[RC_NUM_CHANNELS];
volatile uint16_t RC_SHARED[RC_NUM_CHANNELS];

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    RC_START[channel] = micros();
  } else {
    uint16_t RC_COMPARE = (uint16_t)(micros() - RC_START[channel]);
    RC_SHARED[channel] = RC_COMPARE;
  }
}



// Setup our program
void setup() {
  
  // Set the speed to communicate with the host PC
  Serial.begin(SERIAL_PORT_SPEED);

  // Set our pin modes to input for the pins connected to the receiver
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);

  // Attach interrupts to our pins
  attachInterrupt(digitalPinToInterrupt(RC_CH1_INPUT), READ_RC1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH2_INPUT), READ_RC2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH3_INPUT), READ_RC3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH4_INPUT), READ_RC4, CHANGE);


}


void loop() {
  
  // read the values from our RC Receiver
  rc_read_values();
  
  // output our values to the serial port in a format the plotter can use
  Serial.print(  RC_VALUES[RC_CH1]);  Serial.print(",");
  Serial.print(  RC_VALUES[RC_CH2]);  Serial.print(",");
  Serial.print(  RC_VALUES[RC_CH3]);  Serial.print(",");
  Serial.println(RC_VALUES[RC_CH4]); 
 
}



// Thee functions are called by the interrupts. We send them all to the same place to measure the pulse width
void READ_RC1() { 
   Read_Input(RC_CH1, RC_CH1_INPUT); 
}
void READ_RC2() { 
   Read_Input(RC_CH2, RC_CH2_INPUT);
}
void READ_RC3() { 
   Read_Input(RC_CH3, RC_CH3_INPUT); 
}
void READ_RC4() { 
   Read_Input(RC_CH4, RC_CH4_INPUT); 
}


// This function reads the pulse starts and uses the time between rise and fall to set the value for pulse width
void Read_Input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    RC_START[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - RC_START[channel]);
    RC_SHARED[channel] = rc_compare;
  }
}

// this function pulls the current values from our pulse arrays for us to use. 
void rc_read_values() {
  noInterrupts();
  memcpy(RC_VALUES, (const void *)RC_SHARED, sizeof(RC_SHARED));
  interrupts();
}
