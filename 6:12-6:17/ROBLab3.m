global r m J M I l g Ea N Ra La Km eta Tm

r = 0.045; % Radius of the wheels (m)
m = 0.4; % Combined mas of both wheels and motors (kg)
J = 5e-6; % Mass moment of intertia for m about the axle (kgm^2)
M = 1; % Mass of the robot chassis (kg)
I = 1e-3; % Mass moment of inertia for M about the COM (kgm^2)
l = 0.2; % Distance to the center of gravity from the wheel axle (m)
g = 9.81; % acceleration due to gravity (kg/m^2)

Ea = 12; % Motor input voltage (V)
N = 50; % Gear ratio
Ra = 0.5; % Armature Resistance (Ohms)
La = 1.5e-3; % Arature inductance (H)
Km = 0.05; % Motor speed constant (V/rad/s)
eta = 0.6; % Gearbox efficiency (%)
Tm = 6; % Friction torque (Nm)


A = [0,                                                                                      0, 1, 0
0,                                                                                      0, 0, 1
0, -(M*g*l*(J + M*r^2 + m*r^2 + M*l*r))/(I*J + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2), 0, 0
0,          (M*g*l*(J + M*r^2 + m*r^2))/(I*J + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2), 0, 0];

B = [0;                                                                                                                                   
         0;
         (I + J + M*l^2 + M*r^2 + m*r^2 + 2*M*l*r*cos(0))/(I*J + M^2*l^2*r^2 + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2 - M^2*l^2*r^2*cos(0)^2);
         -(J + M*r^2 + m*r^2 + M*l*r*cos(0))/(I*J + M^2*l^2*r^2 + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2 - M^2*l^2*r^2*cos(0)^2)];

q_0 = [0; 0.3];
qdot_0 = [0; 0.3];

qdq_0 = [0; 0.75; 0 ; 0];
u_0 = 0;

C = eye(4, 4);
D = zeros(4, 1);

eig(A);


Q = diag([10, 1000, 0.0001, 100]);
R = 0.001;

K = lqr(A, B, Q, R)



urmom = eig(A-(B*K))

sys = ss(A, B, C, D);
z = tzero(sys);
p = pole(sys);