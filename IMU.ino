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

uint8_t r = 0.021; // Radius of the wheels (m)
uint8_t m = 0.063; // Combined mas of both wheels and motors (kg)
uint8_t J = 5.56e-5; // Mass moment of intertia for m about the axle (kgm^2)
uint8_t M = 0.279; // Mass of the robot chassis (kg)
uint8_t I = 1e-3; // Mass moment of inertia for M about the COM (kgm^2)
uint8_t l = 0.095; // Distance to the center of gravity from the wheel axle (m)
uint8_t g = 9.81; // acceleration due to gravity (kg/m^2)

uint8_t Ea = 12; // Motor input voltage (V)
uint8_t N = 50; // Gear ratio
uint8_t Ra = 14; // Armature Resistance (Ohms)
uint8_t La = 2.5e-3; // Arature inductance (H)
uint8_t Km = 0.191; // Motor speed constant (V/rad/s)
uint8_t eta = 0.6; // Gearbox efficiency (%)
uint8_t Tm = 0.00955; // Friction torque (Nm)

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

// Define constants
const float T = 1.0;
const float pw = 5.0;
const float pb = 5.0;
const float qw = 100.0;
const float qb = 100.0;
const float rtheta = 5e-5;
const float rw = 0.3;

// Define matrices using the BasicLinearAlgebra library
BLA::Matrix<3, 3> Ad = {1, 0, 0, T, 1, 0, 0, 0, 1};
BLA::Matrix<3, 3> Qd = {T * qw, 0.5 * T * T * qw, 0, 0.5 * T * T * qw, (1.0 / 3) * T * T * T * qw, 0, 0, 0, T * qb};
BLA::Matrix<2, 2> Rd = {rtheta / T, 0, 0, rw / T};
BLA::Matrix<2, 3> Cd = {0, 1, 0, 1, 0, 1};
BLA::Matrix<3, 3> Idontcare = {1, 0, 0, 0, 1, 0, 0, 0, 1};
BLA::Matrix<3, 3> P = {T * pw, 0.5 * pw * T * T, 0, 0, (1.0 / 3) * T * T * T * pw, 0, 0, 0, T * pb};

BLA::Matrix<3> u = {0, 0, 0};
BLA::Matrix<2> y = {0, 0};

BLA::Matrix<1, 4> KingKong = {-0.001, -200, -0.5, -25};

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

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);
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
    // Serial.print("Encoder counts AB: ");
    // Serial.print(encoderAB_counts);
    // Serial.print(" counts, Angle: ");
    // Serial.print(angleAB);
    // Serial.println(" radians");
    // Serial.print("Angular velocity CD: ");
    // Serial.print(angular_velocityCD);
    // Serial.print(" rad/s, Angle: ");
    // Serial.print(angleCD);
    // Serial.print(" radians");

    // uint16_t End_time = millis();

    // uint16_t time_elapsed = End_time - time;

    // Serial.print(" Time Elasped: ");
    // Serial.println(time);

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Serial.print("Accelerometer (m/s^2): ");
    // Serial.print(ax/16384.0); Serial.print(", "); //Scale with 2^-14 to convert readings into +/- 2g
    // Serial.print(ay/16384.0); Serial.print(", ");
    // Serial.println(az/16384.0);
    // Serial.print("Gyroscope (deg/s): ");
    // Serial.print(gx/131.0); Serial.print(", "); //Scale with 250/2^15 to convert readings into +/- 250dps
    // Serial.print(gy/131.0); Serial.print(", ");
    // Serial.println(gz/131.0);

    float yww = gx / 131.0; // Assuming 131 LSB/deg/s for 500 deg/s range
    float ywb = gy / 131.0;
    float ywn = gz / 131.0;

    float yar = ax / 4096.0; // Assuming 4096 LSB/g for 8g range
    float yat = ay / 4096.0;
    float yan = az / 4096.0;

    static BLA::Matrix<3> uPrev = {0, 0, 0};
    static BLA::Matrix<3, 3> P = {T * pw, 0.5 * pw * T * T, 0, 0, (1.0 / 3) * T * T * T * pw, 0, 0, 0, T * pb};

    float yt = atan2(-yar, yat) + yan;
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

    uint8_t theta = u(1);
    uint8_t dtheta = u(0);


    P = (Idontcare - K * Cd) * Pnew;

    uPrev = u;

    // Update last time
    last_time = current_time;

    Matrix<4, 1> X = {angleAB, theta, angular_velocityAB, dtheta};
    // Serial.print("Phi: ");
    // Serial.println(X(0));
    // Serial.print("Theta: ");
    // Serial.println(X(1));
    // Serial.print("DPhi: ");
    // Serial.println(X(2));
    // Serial.print("DTheta: ");
    // Serial.println(X(3));


    Matrix<1, 1> torque = KingKong * X;
    uint8_t tau = torque(0);
    // Serial.print("The torque is: ");
    // Serial.println(torque_val);

    //uint8_t V = MotorVoltage(torque_val, angular_velocityAB);

    uint8_t i = 0;
    uint8_t E = 0;

    if(tau > 0){
      i = ((tau / (eta * N)) - Tm) / Km;
    }
    else if(tau < 0){
      i = ((tau / (eta * N)) + Tm) / Km;
    }
       
    else if (tau == 0){
      E = 0;
    }

    E = (Ra * i) + (Km * N * angular_velocityAB);

    Serial.print("The voltage is: ");
    Serial.print(E);
    Serial.println("Volts");

    uint8_t duty_cycle = (E / 12) * 255;

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

// uint8_t MotorVoltage(uint8_t tau, uint8_t dphi){
//     uint8_t i = 0;
//     uint8_t E = 0;

//     if(tau > 0){
//       i = ((tau / (eta * N)) - Tm) / Km;
//     }
//     else if(tau < 0){
//       i = ((tau / (eta * N)) + Tm) / Km;
//     }
       
//     else if (tau == 0){
//       E = 0;
//     }
//     E = (Ra * i) + (Km * N * dphi);
//     return E;
// }
