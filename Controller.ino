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

#include <BasicLinearAlgebra.h>

//TORQUE IS OBTAINED BY MULTIPLYING -K by X, (1 x 4) * (4 x 1)

BLA::Matrix<4, 1> qdq_0 = {0, 0.3, 0, 0};
BLA::Matrix<1, 4> K = {-0.001, -200, -0.5, -25};

void setup() {

  Serial.begin(9600);
  // put your setup code here, to run once:

  BLA::Matrix<1, 1> torque = K * qdq_0;
  uint8_t torque_val = torque(0);
 

}

void loop() {
  // put your main code here, to run repeatedly:

  // BLA::Matrix<1, 1> torque = K * qdq_0;
  // uint8_t torque_val = torque(0);

  // Serial.println(torque_val);
}
