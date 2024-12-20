#include <Wire.h>
#include <MPU6050.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

MPU6050 mpu;

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
unsigned long baud_rate = 9600;

void setup() {

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  PCMSK0 |= (1 << PCINT5);
  PCMSK0 |= (1 << PCINT6);
  PCICR |= (1 << PCIE0);

  pinMode(ENCODER_C, INPUT_PULLUP);
  pinMode(ENCODER_D, INPUT_PULLUP);
  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoder_isr, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCODER_C), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_D), encoder_isr, CHANGE);
  // Initialize serial communication

  Serial.begin(baud_rate);
  Wire.begin();
  mpu.initialize(); //Initializes MPU for measurements +/-250dps and +/-2g
}

void loop() {

  uint16_t time = millis();

  if(time - previousMillis >= interval){

    previousMillis = time;
    // Calculate the elapsed time since the last encoder pulse
    unsigned long current_time = micros();
    // Calculate the time difference since the last interrupt
    float dt_interrupt = (float)(current_time - last_time) / 1000000;
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
    // Print results
    Serial.print("Encoder counts AB: ");
    Serial.print(encoderAB_counts);
    Serial.print(" counts, Angle: ");
    Serial.print(angleAB);
    Serial.println(" radians");
    Serial.print("Angular velocity CD: ");
    Serial.print(angular_velocityCD);
    Serial.print(" rad/s, Angle: ");
    Serial.print(angleCD);
    Serial.print(" radians");

    // uint16_t End_time = millis();

    // uint16_t time_elapsed = End_time - time;

    Serial.print(" Time Elasped: ");
    Serial.println(time);

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print("Accelerometer (m/s^2): ");
    Serial.print(ax/16384.0); Serial.print(", "); //Scale with 2^-14 to convert readings into +/- 2g
    Serial.print(ay/16384.0); Serial.print(", ");
    Serial.println(az/16384.0);
    Serial.print("Gyroscope (deg/s): ");
    Serial.print(gx/131.0); Serial.print(", "); //Scale with 250/2^15 to convert readings into +/- 250dps
    Serial.print(gy/131.0); Serial.print(", ");
    Serial.println(gz/131.0);
    //delay(1000);


    // Update last time
    last_time = current_time;

    
    Serial.print("Phi: ");
    Serial.println(x(0));
    Serial.print("Theta: ");
    Serial.println(x(1));
    Serial.print("DPhi: ");
    Serial.println(x(2));
    Serial.print("DTheta: ");
    Serial.println(x(3));
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
