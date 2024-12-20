#include <Wire.h>
#include <MPU6050.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

float r = 0.021; // Radius of the wheels (m)
float m = 0.063; // Combined mas of both wheels and motors (kg)
float J = 5.56e-5; // Mass moment of intertia for m about the axle (kgm^2)
float M = 0.279; // Mass of the robot chassis (kg)
float I = 1e-3; // Mass moment of inertia for M about the COM (kgm^2)
float l = 0.095; // Distance to the center of gravity from the wheel axle (m)
float g = 9.81; // acceleration due to gravity (kg/m^2)

float Ea = 12; // Motor input voltage (V)
float N = 50; // Gear ratio 
float Ra = 14; // Armature Resistance (Ohms)
float La = 2.5e-3; // Arature inductance (H)
float Km = 0.191; // Motor speed constant (V/rad/s)
float eta = 0.6; // Gearbox efficiency (%), could also be around 40%
float Tm = 0.00955; // Friction torque (Nm)
float dead_zone = 0.05;

float ia;
float E;
float theta;
float dtheta;
float duty_cycle;
float V;
float Aldehyde = 0.1;

//BLA::Matrix<4, 1> qdq_0 = {0, 0.3, 0, 0};
BLA::Matrix<1, 4> Ketone = {-0.01, -3200, -0.5, -25};
//10, -8e3, 108, 83


#define PI 3.1415926535897932384626433832795 //028841971693993751

#define M1A 7
#define M1B 6
#define M2A 10
#define M2B 9
//define and establish the ports that will be used to control the motors

//-------------------------------------------------------------------------------------------------------------------------//

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
unsigned long interval = 10;
unsigned long previousMillis = 0;

//-------------------------------------------------------------------------------------------------------------------------//

//define and initialize the parameters for the counter
#undef F_CPU
#define F_CPU 16000000UL // 16MHz clock frequency
#define PRESCALER 8UL
#define F_S 100UL  // 1Hz sampling frequency

volatile uint8_t countFlag = 0;
int counts = F_CPU/(PRESCALER*F_S) - 1; //
volatile bool Mode = 0;
unsigned int i;

//-------------------------------------------------------------------------------------------------------------------------//

//initiliaze parameters for our Kalman Filter

MPU6050 mpu;

// Define constants
const float T = 0.01;
const float pw = 1000;
const float pb = 10;
const float qw = 100000000000000;
const float qb = 1;
const float rtheta = 100;
const float rw = 100;

// Define matrices using the BasicLinearAlgebra library
BLA::Matrix<3, 3> Ad = {1, 0, 0, T, 1, 0, 0, 0, 1};
BLA::Matrix<3, 3> Qd = {T * qw, 0.5 * T * T * qw, 0, 0.5 * T * T * qw, (1.0 / 3) * T * T * T * qw, 0, 0, 0, T * qb};
BLA::Matrix<2, 2> Rd = {rtheta / T, 0, 0, rw / T};
BLA::Matrix<2, 3> Cd = {0, 1, 0, 1, 0, 1};
BLA::Matrix<3, 3> Id = {1, 0, 0, 0, 1, 0, 0, 0, 1};
BLA::Matrix<3, 3> P = {T * pw, 0.5 * pw * T * T, 0, 0, (1.0 / 3) * T * T * T * pw, 0, 0, 0, T * pb};

BLA::Matrix<3> u = {0, 0, 0};
BLA::Matrix<2> y = {0, 0};

float MAX_voltage = 7.2;

float MotorVoltage(float tau, float dphi){

  if(tau > 0 && tau > Tm){
    ia = ((tau / (eta * N)) - Tm) / Km;
  }
  else if(tau < 0 && tau < Tm){
    ia = ((tau / (eta * N)) + Tm) / Km;
  }
      
  else {
    E = 0;
  }

  E = (Ra * ia) + (Km * N * dphi);

  if(E > MAX_voltage){
    E = MAX_voltage;
  }
  else if (E < -MAX_voltage){
    E = -MAX_voltage;
  }
  return E;
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

ISR(TIMER5_COMPA_vect) {  
  countFlag = 1;
  Mode = !Mode;
}

void setup() {
  //This is here to initialize the parameters for the motors:
  Serial.begin(9600);
  
  pinMode(M1A, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2B, OUTPUT);

  //-------------------------------------------------------------------------------------------------------------------------//

//this next part is to initialize the gyroscope:
  Wire.begin();
  mpu.initialize(); //Initializes MPU for measurements +/-250dps and +/-2g

//-------------------------------------------------------------------------------------------------------------------------//

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


  /*
     Timers and interrupts
  */
  // Set a 1Hz Timer for clocking controller
  TCCR5A = 0;
  TCCR5B = (1 << WGM52) | (0 << CS52) | (1 << CS51) | (0 << CS50);
  OCR5A = F_CPU / (PRESCALER * F_S) - 1; //Set the number of counts to compare the counter with
  TIMSK5 |= (1 << OCIE5A);
  TCNT5 = 0;

}

void loop() {
  // put your main code here, to run repeatedly:

  //-------------------------------------------------------------------------------------------------------------------------//

  //this is for the encoder

  int16_t ax, ay, az, gx, gy, gz;

  unsigned long current_time = micros();
  float dt_interrupt = (float)(current_time - last_time) / 1000000;

  uint16_t time = millis();
  
  // unsigned long current_time = micros();
  // float dt_interrupt = (float)(current_time - last_time) / 1000000;
  

  if(time - previousMillis > interval){

    previousMillis = time;
    // Calculate the elapsed time since the last encoder pulse

    // int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Calculate the time difference since the last interrup
    // Calculate the change in the position of the encoder in radians
    position_changeAB = 2 * PI * encoderAB_counts / COUNTS_PER_REVOLUTION;
    // Reset encoder counts
    encoderAB_counts = 0;
    // Calculate the angular velocity
    angular_velocityAB = position_changeAB / dt_interrupt;
    // Calculate the angle
    angleAB += position_changeAB;

    // Update last time
    last_time = current_time;
  }

  float yww = (gx * (PI / 180)) / 131.0; // Assuming 131 LSB/deg/s for 500 deg/s range

  float yar = ax / 16384.0; // Assuming 4096 LSB/g for 8g range
  float yat = ay / 16384.0;
  float yan = az / 16384.0;

  static BLA::Matrix<3> uPrev = {0, 0, 0};
  static BLA::Matrix<3, 3> P = {T * pw, 0.5 * pw * T * T, 0, 0, (1.0 / 3) * T * T * T * pw, 0, 0, 0, T * pb};

  float yt = atan2(yan, -yat); // - (PI / 2);
  y(0) = yww;
  y(1) = yt;

  BLA::Matrix<3> uPredict = Ad * uPrev;

  BLA::Matrix<3, 3> Pnew = Ad * P * (~Ad) + Qd;

  BLA::Matrix<3, 2> CdT = ~Cd; // Transpose Cd to match dimensions

  BLA::Matrix<2, 2> S = Cd * Pnew * CdT + Rd;

  BLA::Matrix<3, 2> K = Pnew * CdT * Inverse(S);

  BLA::Matrix<2> yPredict = Cd * uPredict;

  BLA::Matrix<2> yDiff = y - yPredict;

  u = uPredict + K * yDiff;

  P = (Id - K * Cd) * Pnew;

  uPrev = u;

  theta = u(0);
  dtheta = u(1);

  //-------------------------------------------------------------------------------------------------------------------------//

  //estimated states x [phi, theta, dphi, dtheta]
  Matrix<4, 1> x = {angleAB, theta, angular_velocityAB, dtheta};

  Serial.print("Phi: ");
  Serial.println(x(0));
  Serial.print("Theta: ");
  Serial.println(x(1));
  Serial.print("Dphi: ");
  Serial.println(x(2));
  Serial.print("Dtheta: ");
  Serial.println(x(3));

  BLA::Matrix<1, 1> torque = (Ketone * Aldehyde) * x;
  float tau = torque(0) / 2;
  Serial.print("Torque: ");
  Serial.println(tau);

  //-------------------------------------------------------------------------------------------------------------------------//

  V = MotorVoltage(tau, angular_velocityAB);
  Serial.println("Voltage: ");
  Serial.println(V);

  float duty_cycle = (abs(V) / MAX_voltage) * 255;
  float SLOW_DOWN = 150;

  if(x(1) < dead_zone && x(1) > -dead_zone){
    analogWrite(M1A, 0);
    analogWrite(M1B, 0);
    analogWrite(M2A, 0);
    analogWrite(M2B, 0);
  }
  else if(V > 0){
    analogWrite(M1A, duty_cycle);
    analogWrite(M1B, SLOW_DOWN);

    analogWrite(M2A, duty_cycle);
    analogWrite(M2B, SLOW_DOWN);
  }

  else if(V < 0){
    analogWrite(M1A, SLOW_DOWN);
    analogWrite(M1B, duty_cycle);
    
    analogWrite(M2A, SLOW_DOWN);
    analogWrite(M2B, duty_cycle);
  }
  else{
    analogWrite(M1A, 0);
    analogWrite(M1B, 0);
    analogWrite(M2A, 0);
    analogWrite(M2B, 0);
  }

}
