/*
  Arduino RC Controller Input
  ---------------------------
  Lesson 2: Refine and Translate RC Controller Input 

  Adds the following features:

    Failsafe    - if the RC controller goes out of range, react
    Inversion   - if your control stick goes the wrong way we can handle it
    Deadzone    - dont react until the sticks are intentionally pushed
    Translation - map our values to a sensible range like -100 to 100


  Read more about this example on the blog post at https://www.andyvickers.net/

  Author: Andy Vickers
  Downloaded From: https://github.com/andyman198/ArduinoRCControllerInput
  
*/

// Set the port speed for host communication
#define SERIAL_PORT_SPEED 9600

// Set the size of the arrays (increase for more channels)
#define RC_NUM_CHANNELS 5

// Set up our receiver channels - these are the channels from the receiver
#define RC_CH1 0  // Right Stick LR
#define RC_CH2 1  // Right Stick UD
#define RC_CH3 2  // Left  Stick UD
#define RC_CH4 3  // Left  Stick LR
#define RC_CH5 4  // Left  Stick LR


// Set up our channel pins - these are the pins that we connect to the receiver
// use this website as a guide:
//    https://www.arduino.cc/reference/cs/language/functions/external-interrupts/attachinterrupt/


// ARDUINO MEGA
//#define RC_CH1_INPUT  18 // receiver pin 1
//#define RC_CH2_INPUT  19 // receiver pin 2
//#define RC_CH3_INPUT  20 // receiver pin 3
//#define RC_CH4_INPUT  21 // receiver pin 4
//#define RC_CH4_INPUT  2 // receiver pin 5
//#define RC_CH4_INPUT  3 // receiver pin 6


// ARDUINO Nano 33 IoT
// #define RC_CH1_INPUT 2   // receiver pin 1
// #define RC_CH2_INPUT 3   // receiver pin 2
// #define RC_CH3_INPUT 4   // receiver pin 3
// #define RC_CH4_INPUT 5   // receiver pin 4
// #define RC_CH5_INPUT 6   // receiver pin 5
// #define RC_CH5_INPUT 7   // receiver pin 6
// #define RC_CH5_INPUT 8   // receiver pin 7
// #define RC_CH5_INPUT 9   // receiver pin 8
// #define RC_CH5_INPUT 10  // receiver pin 9
// #define RC_CH5_INPUT 11  // receiver pin 10
// #define RC_CH5_INPUT 12  // receiver pin 11
// #define RC_CH5_INPUT 13  // receiver pin 12

// ARDUINO PRO MICRO / LEONARDO
#define RC_CH1_INPUT 2  // receiver pin 1
#define RC_CH2_INPUT 3  // receiver pin 2
#define RC_CH3_INPUT 1  // receiver pin 3
#define RC_CH4_INPUT 0  // receiver pin 4
#define RC_CH5_INPUT 7  // receiver pin 5


// Set up some arrays to store our pulse starts and widths
uint16_t RC_VALUES[RC_NUM_CHANNELS];
uint32_t RC_START[RC_NUM_CHANNELS];
volatile uint16_t RC_SHARED[RC_NUM_CHANNELS];


// to the extent possible, all the variables are scaleable, you will need to add more values
// here if you use a higher number of channels
uint16_t RC_LOW[RC_NUM_CHANNELS] = { 1092, 1092, 1092, 1092, 1092 };
uint16_t RC_MID[RC_NUM_CHANNELS] = { 1508, 1508, 1508, 1508, 0 };
uint16_t RC_HIGH[RC_NUM_CHANNELS] = { 1924, 1924, 1924, 1924, 1924 };

// The RC Channel mode helps us know how to use and refine the signal
// Settings are:
// 0 = a joystick with a centerpoint (deadzone in middle)
// 1 = a throttle that goes from low to high (deadzone at start)
// 2 = a a switch (either on or off)
uint16_t RC_CHANNEL_MODE[RC_NUM_CHANNELS] = { 0, 0, 0, 0, 1 };

// Do we need to invert the input? 1 = yes, 0 = no
// Use this if your joystick goes the wrong way
uint16_t RC_INVERT[RC_NUM_CHANNELS] = { 1, 0, 1, 0, 1 };

// what should we set the value to if we cant see the RC controller?
uint16_t RC_FAILSAFE_VALUE[RC_NUM_CHANNELS] = { 1508, 1508, 1508, 1508, 1092 };

// a place to store our mapped values
float RC_TRANSLATED_VALUES[RC_NUM_CHANNELS];

// some boundaries for our mapped values
float RC_TRANSLATED_LOW[RC_NUM_CHANNELS] = { -100, -100, -100, -100, 0 };
float RC_TRANSLATED_MID[RC_NUM_CHANNELS] = { 0, 0, 0, 0, 0 };
float RC_TRANSLATED_HIGH[RC_NUM_CHANNELS] = { 100, 100, 100, 100, 100 };

// What percentage deadzone is allowed? values in percent e.g. 10 = 10%
uint16_t RC_DZPERCENT[RC_NUM_CHANNELS] = { 30, 30, 30, 30, 5 };


// Setup our program
void setup() {

  // Set the speed to communicate with the host PC
  Serial.begin(SERIAL_PORT_SPEED);

  // Set our pin modes to input for the pins connected to the receiver
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);

  // Attach interrupts to our pins
  attachInterrupt(digitalPinToInterrupt(RC_CH1_INPUT), READ_RC1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH2_INPUT), READ_RC2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH3_INPUT), READ_RC3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH4_INPUT), READ_RC4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH5_INPUT), READ_RC5, CHANGE);
}

unsigned long current = 0;
unsigned long prev = 0;
const unsigned long interval = 100000UL;

bool failsafeActive = true;

void loop() {

  // read the values from our RC Receiver
  rc_read_values();

  // first lets see if we are in a failsafe condition
  processFailsafe();

  // invert the signal if we want to (e.g. the joystick goes the wrong way)
  rc_invert_values();

  // only make changes to the signal if we arent in a failsafe condition
  if (failsafeActive == false) {
    rc_deadzone_adjust();
  }

  // map the radio values to the range we want e.g. -100 to 100
  rc_translate_values();


  // Now its over to you!
  /* 
     Now you can use the newly translated values in your project, you can access them like this:

     Channel 1 = RC_TRANSLATED_VALUES[0]
     Channel 2 = RC_TRANSLATED_VALUES[1]
     Channel 3 = RC_TRANSLATED_VALUES[2]
     etc
 
     assuming you set them to a range you can use, you could position a servo
     or drive a motor or even pass them through to an odrive or ROS

                here's an example:
                #include <Servo.h>
                Servo servo1; int servoPin1 = 9;
                Servo servo2; int servoPin1 = 10;
                Servo servo3; int servoPin1 = 11;

                void setup(){
                  servo1.attach(servoPin1);
                  servo2.attach(servoPin2);
                  servo3.attach(servoPin3);
                }

                void loop(){
                  servo1.write(RC_TRANSLATED_VALUES[0]);
                  delay(1000);
                  servo2.write(RC_TRANSLATED_VALUES[1]);
                  delay(1000);
                  servo3.write(RC_TRANSLATED_VALUES[2]);
                  delay(1000);
                }

  */
  


  // keep track of time
  current = micros();

  // This is our plotter Chart output, we only do it every so often or the plotter moves too fast
  if (current - prev >= interval) {
    prev += interval;

    // loop through all channels and display value
    // note: the plotter only holds 8 values, if you add more than 5 plus the min, mid and max here it
    // will not show all your plots!

    for (int i = 0; i < RC_NUM_CHANNELS; i++) {
      Serial.print("CH");
      Serial.print(i);
      Serial.print(":");
      Serial.print(RC_TRANSLATED_VALUES[i]);
      Serial.print(",");
    }

    //Use Ch1 as a reference point for low and high and plot them
    Serial.print("LOW:");
    Serial.print(RC_TRANSLATED_LOW[0]);
    Serial.print(",");
    Serial.print("HIGH:");
    Serial.print(RC_TRANSLATED_HIGH[0]);

    // if failsafe is active, display on the serial but also plot it as a visual aid
    if (failsafeActive == true) {
      Serial.println(",FAILSAFE:2500");
    } else {
      Serial.println("");
    }
  }
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
void READ_RC5() {
  Read_Input(RC_CH5, RC_CH5_INPUT);
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

void rc_invert_values() {

  // loop through the channels

  for (int i = 0; i < RC_NUM_CHANNELS; i++) {
    // do we need to invert?
    if (RC_INVERT[i] == 1) {

      if (RC_CHANNEL_MODE[i] == 0) {

        // if this is a joystick with a midpoint
        RC_VALUES[i] = (RC_HIGH[i] + RC_LOW[i]) - RC_VALUES[i];

      } else if (RC_CHANNEL_MODE[i] == 1) {

        // if this is a throttle
        RC_VALUES[i] = RC_HIGH[i] - (RC_VALUES[i] - RC_LOW[i]);
      }
    }

    // a little clipping to make sure we dont go over or under the bounds

    // clip the high range so it doesnt go over the max
    if (RC_VALUES[i] > RC_HIGH[i]) {
      RC_VALUES[i] = RC_HIGH[i];
    }

    // clip the low range so it doesnt go under the min
    if (RC_VALUES[i] < RC_LOW[i]) {
      RC_VALUES[i] = RC_LOW[i];
    }
  }
}

void rc_translate_values() {

  // Loop through all our channels
  for (int i = 0; i < RC_NUM_CHANNELS; i++) {

    // translate the RC channel value into our new number range
    RC_TRANSLATED_VALUES[i] = translateValueIntoNewRange((float)RC_VALUES[i], (float)RC_HIGH[i], (float)RC_LOW[i], RC_TRANSLATED_HIGH[i], RC_TRANSLATED_LOW[i]);
  }
}


void processFailsafe() {

  //temporarily reset failsafe while we calculate where we are
  failsafeActive = false;

  // see if any channels have failed
  for (int i = 0; i < RC_NUM_CHANNELS; i++) {

    // Lets convert our range into -100 to 100 so we can compare against our deadzone percent
    float newval = translateValueIntoNewRange((float)RC_VALUES[i], (float)RC_HIGH[i], (float)RC_LOW[i], 100.0, 0);

    if (abs(newval) > 105.0) {

      //failsafe active, we are way out of range of where we should be, likely the controller
      // lost battery or went out of range
      failsafeActive = true;
    }
  }

  // if we triggered a failsafe, we need to set all the channels to their failsafe value
  // in my experience, only channel 1 goes out of bounds in these situations
  if (failsafeActive == true) {
    for (int i = 0; i < RC_NUM_CHANNELS; i++) {
      RC_VALUES[i] = RC_FAILSAFE_VALUE[i];
    }
  }
}

void rc_deadzone_adjust() {

  // Lets convert our range into -100 to 100 so we can compare against our deadzone percent
  for (int i = 0; i < RC_NUM_CHANNELS; i++) {
    // first off, we cant divide by zero so lets get that out the way

    float newval = 0;

    if (RC_CHANNEL_MODE[i] == 0) {
      // if this is a joystick with a midpoint, our deadzone should be around the middle
      newval = translateValueIntoNewRange((float)RC_VALUES[i], (float)RC_HIGH[i], (float)RC_LOW[i], 100.0, -100.0);

      if (abs(newval) < RC_DZPERCENT[i]) {
        // reset to the midpoint if we are in the deadzone
        RC_VALUES[i] = RC_MID[i];
      }

    } else if (RC_CHANNEL_MODE[i] == 1) {
      // if this is a throttle, our deadzone should be at the low point
      newval = translateValueIntoNewRange((float)RC_VALUES[i], (float)RC_HIGH[i], (float)RC_LOW[i], 100.0, 0.0);

      if (abs(newval) < RC_DZPERCENT[i]) {
        // reset to the low point if we are in the deadzone
        RC_VALUES[i] = RC_LOW[i];
      }
    }
  }
}

float translateValueIntoNewRange(float currentvalue, float currentmax, float currentmin, float newmax, float newmin) {
  // Use this formula to work out where we are in the new range
  // NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
  //
  // this formula was lovingly stolen from https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio

  return (((currentvalue - currentmin) * (newmax - newmin)) / (currentmax - currentmin)) + newmin;
}