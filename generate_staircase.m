
%% Generate staircase.mat for QUBE-Servo 3

clc;
clear;
close all;

% Time vector
dt = 0.001;
t = (0:dt:6)';

% Staircase voltage input
levels = [0 0.5 1 1.5 2 2.5 3 ...
          2.5 2 1.5 1 0.5 0];

u = zeros(size(t));

for k = 1:length(levels)

    idx = t >= (k-1)*0.5 & t < k*0.5;

    u(idx) = levels(k);

end

% Identified QUBE parameters
K = 16.294;
tau = 0.0983;

% Simulate angular velocity
w_meas = zeros(size(t));

for k = 1:length(t)-1

    a = exp(-dt/tau);

    w_meas(k+1) = a*w_meas(k) ...
        + K*(1-a)*u(k);

end

% Save MATLAB data file
save('staircase.mat','t','u','w_meas');

% Plot staircase response
figure;

subplot(2,1,1);
plot(t,u,'LineWidth',1.5);
grid on;
ylabel('Voltage (V)');
title('Staircase Input');

subplot(2,1,2);
plot(t,w_meas,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Simulated Staircase Response');

fprintf('staircase.mat generated successfully!\n');
