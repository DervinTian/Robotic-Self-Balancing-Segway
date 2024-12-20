global p

p.M = 1; % Mass of the robot chassis in kg
p.m = 0.4; % Combined mass of both wheels and motors in kg
p.r = 0.045; %  Radius of the wheels in meters
p.g = 9.81; %  Acceleration due to gravity in m/s^2
p.I = 1e-3; % Mass moment of inertia for M about the COM in kg/m^2
p.J = 5e-6; % Mass moment of inertia for m about the axle in kg/m^2
p.l = 0.2; % Distance to the center of gravity from the wheel axle in meters
