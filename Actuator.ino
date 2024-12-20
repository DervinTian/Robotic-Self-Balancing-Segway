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
float eta = 0.6; // Gearbox efficiency (%)
float Tm = 0.00955; // Friction torque (Nm)

float i;
float E;

//BLA::Matrix<4, 1> qdq_0 = {0, 0.3, 0, 0};
BLA::Matrix<1, 4> Ketone = {-0.001, -200, -0.5, -25};


#define PI 3.1415926535897932384626433832795 //028841971693993751

#define M1A 7
#define M1B 6
#define M2A 10
#define M2B 9
//define and establish the ports that will be used to control the motors

//-------------------------------------------------------------------------------------------------------------------------//

//initiliaze parameters for our Kalman Filter

MPU6050 mpu;

float V = 12;


float MotorVoltage(float tau, float dphi){
  Serial.println("WHAT A WONDERFUL WORLD, I am now here");

  if(tau > 0){
    Serial.print("Tau is: ");
    Serial.println(tau);
    i = ((tau / (eta * N)) - Tm) / Km;
    Serial.print("O: ");
    Serial.println(i);
    delay(1000);
  }
  else if(tau < 0){
    i = ((tau / (eta * N)) + Tm) / Km;
    Serial.println(i);
    delay(1000);
  }
      
  else if (tau == 0){
    E = 0;
    Serial.print("NUMA NUMA NUMA YEI, KEE PU TAL, SIE DRAGOSTEA DIN TEI");
  }

  E = (Ra * i) + (Km * N * dphi);
  return E;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(M1A, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2B, OUTPUT);

  //-------------------------------------------------------------------------------------------------------------------------//

    // Initialize serial communication
    Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print("Hello World: ");
  // float tau = -15;
  // float angular_velocityAB = -15;

  float duty_cycle = (abs(V) / 12) * 255;

  Serial.print("I'm at the top mom! ");


  if(V > 0){
    analogWrite(M1A, duty_cycle);
    analogWrite(M1B, 0);

    analogWrite(M2A, duty_cycle);
    analogWrite(M2B, 0);
  }

  else if(V < 0){
    analogWrite(M1A, 0);
    analogWrite(M1B, duty_cycle);
    
    analogWrite(M2A, 0);
    analogWrite(M2B, duty_cycle);
  }
  else{
    analogWrite(M1A, 0);
    analogWrite(M1B, 0);
    analogWrite(M2A, 0);
    analogWrite(M2B, 0);
  }

  // V = V * -1;

  Serial.println("I made it the bottom mom!");

  delay(50);

}
