Data = readmatrix("AcceleratorLeft.xlsx");

gyro_data = Data(1:2:end , 2:end);
accel_data = Data(2:2:end , 2:end);

% figure(1);
% plot(gyro_data);
% title("Gyro");

%figure(2);
%plot(accel_data);
%title("Accel");

