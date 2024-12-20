clear all
global pomega pb qb qomega romega rtheta

Data = readmatrix("AcceleratorLeft.xlsx");

gyro_data = Data(1:2:end , 2:end);
accel_data = Data(2:2:end , 2:end);

pomega = 10000; %variance between omega
pb = 10; %variance in bias
qb = 1;%1e5; % 
qomega = 100000000000000;
romega = 10000; %1e-5;
rtheta = 10000;%1e-5;

mu_b = [0, 0, 0];
mu_omega = accel_data;
mu_theta = gyro_data;
mu_0 = [0; 0; 0];
mu_prev = mu_0;

T = 1;

Ad = [1, 0, 0;
      T, 1, 0;
      0, 0, 1];

Qd = [T * qomega, 0.5 * T^2 * qomega, 0;
      0.5 * T^2 * qomega, (1/3) * T^3 * qomega, 0;
      0, 0, T * qb]; 

Q = [qomega, 0, 0;
    0, 0, 0;
    0, 0, qb]; %Model Accuracy

P = [T * pomega, 0.5 * T^2 * pomega, 0;
     0.5 * T^2 * pomega, (1/3) * T^3 * pomega, 0;
     0, 0, T * pb]; %Initial Covariance Matrix

P_prev = P;

R = [rtheta, 0;
     0, romega]; %Sensor Accuracy

Rd = (1/T) * R;

Cd = [0, 1, 0; 1, 0, 1];

I = eye(3, 3);

mus = zeros(3, 1);

mus_omega = zeros(1, 1);

mus_theta = zeros(1, 1);

ys = zeros(2, 1);

yomegas = zeros(1, 1);

ythetas = zeros(1, 1);

for i = 1:72

    omegat = gyro_data(i , 1) * (pi / 180);
    % bt = gyro_data(i , 2) * (pi / 180);
    % vt = gyro_data(i , 3) * (pi / 180);
    
    yomega = omegat;
    
    ar = accel_data(i , 1);
    at = accel_data(i , 2);
    %avt = accel_data(i , 3);

    ytheta = atan2(-ar, at);

    yk = [omegat; ytheta];

    %yk = Cd * y;

    mu_pred = Ad * mu_prev;

    P_pred = (Ad * P_prev * transpose(Ad)) + Qd;

    Kk = (P_pred * transpose(Cd)) * inv((Cd * P_pred * transpose(Cd)) + Rd);

    mu_k = mu_pred + (Kk * (yk - (Cd * mu_pred))); %Predicted values

    Pk = (I - (Kk * Cd)) * P_pred; %Covriance

    mu_prev = mu_pred;
    P_prev = P_pred;

    mus = vertcat(mus, mu_k); %predicted values from Kalman filter

    ys = vertcat(ys, yk); %actual values from Kalman filter

    mus_omega = vertcat(mus_omega, mu_k(2, 1));

    mus_theta = vertcat(mus_theta, mu_k(1, 1));

    yomegas = vertcat(yomegas, yomega);

    ythetas = vertcat(ythetas, ytheta);

end

% figure
% plot(mus)
% hold on;
% plot(ys)
% title("Predicted Mu's vs Actual Y's")
% legend('Predicted Mu','Actual Y')

figure
hold on;
% subplot(2, 1, 1)
plot(mus_omega);
%title("Predicted Omega");

%hold on;
plot(yomegas, "-.", LineWidth=1.5);


%title("Measured Omegas")

% subplot(2, 1, 2)
plot(mus_theta);
%title("Predicted Thetas");

%hold on;
plot(ythetas,  "-.", LineWidth=1.5);
%title("Measured Thetas")

legend("Predicted Omega", "Measured Omegas", "Predicted Thetas", "Measured Thetas")


