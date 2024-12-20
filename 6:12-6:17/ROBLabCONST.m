global r m J M I l g Ea N Ra La Km eta Tm
r = 0.021; % Radius of the wheels (m)
m = 0.063; % Combined mas of both wheels and motors (kg)
J = 5.56e-5; % Mass moment of intertia for m about the axle (kgm^2)
M = 0.279; % Mass of the robot chassis (kg)
I = 1e-3; % Mass moment of inertia for M about the COM (kgm^2)
l = 0.095; % Distance to the center of gravity from the wheel axle (m)
g = 9.81; % acceleration due to gravity (kg/m^2)

Ea = 12; % Motor input voltage (V)
N = 50; % Gear ratio
Ra = 14; % Armature Resistance (Ohms)
La = 2.5e-3; % Arature inductance (H)
Km = 0.191; % Motor speed constant (V/rad/s)
eta = 0.6; % Gearbox efficiency (%)
Tm = 0.00955; % Friction torque (Nm)


A = [0,                                                                                      0, 1, 0
0,                                                                                      0, 0, 1
0, -(M*g*l*(J + M*r^2 + m*r^2 + M*l*r))/(I*J + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2), 0, 0
0,          (M*g*l*(J + M*r^2 + m*r^2))/(I*J + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2), 0, 0];

B = [0;                                                                                                                                   
         0;
         (I + J + M*l^2 + M*r^2 + m*r^2 + 2*M*l*r*cos(0))/(I*J + M^2*l^2*r^2 + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2 - M^2*l^2*r^2*cos(0)^2);
         -(J + M*r^2 + m*r^2 + M*l*r*cos(0))/(I*J + M^2*l^2*r^2 + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2 - M^2*l^2*r^2*cos(0)^2)];

q_0 = [0; 0.1];
qdot_0 = [0; 0];

qdq_0 = [0; 0.3; 0; 0];
u_0 = 0;

ref = [0; 0; 0; 0];


C = eye(4, 4);
Copycat = [0, 1, 0, 0];
yomama = Copycat * 1;
D = zeros(4, 1);

eig(A);


Q = diag([1e0, 1e2, 1e-3, 1e1]);
R = 1e0;

%K = [1, 1, 1, 1];
[K,S,CLP] = lqr(A, B, Q, R);
%K = [-100, -3400, -600, -6500];
%K = [-0.03, -37, -0.4, -12];
K = [-0.001, -200, -0.5, -25];
%K = [-1, -34, -0.6, -6.5];
%K = [-0.00001, -1.582, -0.00036616, -0.0992];

urmom = eig(A-(B*K))

sys = ss(A, B, C, D);
z = tzero(sys);
p = pole(sys);

Aa = [0, 0, 1, 0, 0;
      0, 0, 0, 1, 0;
      0, -(M*g*l*(J + M*r^2 + m*r^2 + M*l*r))/(I*J + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2), 0, 0, 0;
      0, (M*g*l*(J + M*r^2 + m*r^2))/(I*J + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2), 0, 0, 0;
      0, 1, 0, 0, 0];

Ba = [B; 0];

Qa = diag([0.35, 153, 3, 123, 5]);
Rab = 0.1;

Kz1 = lqr(Aa, Ba, Qa, Rab);
Kz = Kz1(5)
%Kz = -5.5;