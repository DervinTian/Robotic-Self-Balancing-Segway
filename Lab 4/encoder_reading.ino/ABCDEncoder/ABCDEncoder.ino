#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;

// Define the pins for the encoder
#define ENCODER_A 2
#define ENCODER_B 3
#define ENCODER_C 11
#define ENCODER_D 12
volatile uint8_t encoderALast;
volatile uint8_t encoderBLast;
volatile uint8_t encoderCLast;
volatile uint8_t encoderDLast;

// Define constants
const float WHEEL_RADIUS = 0.045;  // Radius of the wheel in meters
const float GEAR_RATIO = 50.0;  // Gear ratio of the encoder
const int ENCODER_PULSE_PER_REVOLUTION = 28;  // Number of encoder pulses per revolution
const float COUNTS_PER_REVOLUTION = ENCODER_PULSE_PER_REVOLUTION * GEAR_RATIO; // Pulses per revolution
// Define the variables for the encoder
volatile int16_t encoderAB_counts = 0;
volatile int16_t encoderCD_counts = 0;
volatile unsigned long last_time = 0;
volatile float position_changeAB = 0.0;
volatile float position_changeCD = 0.0;
volatile float angular_velocityAB = 0.0;
volatile float angular_velocityCD = 0.0;
volatile float angleAB = 0.0;
volatile float angleCD = 0.0;
unsigned long interval = 500;
unsigned long previousMillis = 0;

void setup() {
  // Set the encoder pins as inputs and enable pull-up resistors
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_C, INPUT_PULLUP);
  pinMode(ENCODER_D, INPUT_PULLUP);

  PCMSK0 |= (1 << PCINT5);
  PCMSK0 |= (1 << PCINT6);
  PCICR |= (1 << PCIE0);

  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoder_isr, CHANGE);

  // attachInterrupt(digitalPinToInterrupt(ENCODER_C), encoder_ISR, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_D), encoder_ISR, CHANGE);

  // Wire.begin();
  // mpu.initialize();
  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {

    // Calculate the elapsed time since the last encoder pulse
  unsigned long current_time = micros();
  // Calculate the time difference since the last interrupt
  float dt_interrupt = (float)(current_time - last_time) / 1000000;

  
  uint16_t time = millis();

  if(time - previousMillis >= interval){

    previousMillis = time;
    // Calculate the change in the position of the encoder in radians
    position_changeAB = 2 * PI * encoderAB_counts / COUNTS_PER_REVOLUTION;
    position_changeCD = 2 * PI * encoderCD_counts / COUNTS_PER_REVOLUTION;
    // Reset encoder counts
    encoderAB_counts = 0;
    encoderCD_counts = 0;
    // Calculate the angular velocity
    angular_velocityAB = position_changeAB / dt_interrupt;
    angular_velocityCD = position_changeCD / dt_interrupt;
    // Calculate the angle
    angleAB += position_changeAB;
    angleCD += position_changeCD;

    // Update last time
    last_time = current_time;
  }
}

void encoder_isr() {
  uint8_t encoderAState = digitalRead(ENCODER_A);
  uint8_t encoderBState = digitalRead(ENCODER_B);

    if (encoderALast*encoderBLast) { //State 11
    encoderAB_counts -= (encoderAState - encoderBState); 
  } else if (encoderALast && (encoderALast^encoderBLast)) { //State 10
    encoderAB_counts += encoderAState*encoderBState - (1-encoderAState|encoderBState); 
  } else if (encoderBLast && (encoderBLast^encoderALast)) { //State 01
    encoderAB_counts += (1-encoderAState|encoderBState) - encoderAState*encoderBState;
  } else { //State 00
    encoderAB_counts += encoderAState - encoderBState; 
  }

  encoderALast = encoderAState;
  encoderBLast = encoderBState;
}

ISR(PCINT0_vect){
  uint8_t encoderCState = digitalRead(ENCODER_C);
  uint8_t encoderDState = digitalRead(ENCODER_D);
    if (encoderCLast*encoderDLast) { //State 11
    encoderCD_counts -= (encoderCState - encoderDState); 
  } else if (encoderCLast && (encoderCLast^encoderDLast)) { //State 10
    encoderCD_counts += encoderCState*encoderDState - (1-encoderCState|encoderDState); 
  } else if (encoderDLast && (encoderDLast^encoderCLast)) { //State 01
    encoderCD_counts += (1-encoderCState|encoderDState) - encoderCState*encoderDState;
  } else { //State 00
    encoderCD_counts += encoderCState - encoderDState; 
  }

  encoderCLast = encoderCState;
  encoderDLast = encoderDState;
}

