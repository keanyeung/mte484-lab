
% Load saved simulation results
load('lab2a_data.mat');

% Extract signals
ref = out.theta_ref_sat;
theta = out.theta;
Vcmd = out.Vcmd;

% Closed-loop performance
G = tf(K1, [tau 1 0]);
T = feedback(C1*G, 1);

info = stepinfo(T, 'SettlingTimeThreshold', 0.02);

settlingTime = info.SettlingTime;
overshoot = info.Overshoot;
steadyStateError = abs(1 - dcgain(T));
peakVoltage = max(abs(Vcmd.Data(:)));

% Display results
fprintf('Settling Time: %.4f s\n', settlingTime);
fprintf('Overshoot: %.2f %%\n', overshoot);
fprintf('Steady-State Error (unit step): %.6f\n', steadyStateError);
fprintf('Peak Motor Voltage: %.4f V\n', peakVoltage);

% Figure 1: Gear angle tracking
figure(1);
plot(ref.Time, ref.Data, 'k--', 'LineWidth', 1.5);
hold on;
plot(theta.Time, theta.Data, 'b', 'LineWidth', 1.5);
hold off;

xlabel('Time (s)');
ylabel('Gear Angle (rad)');
title('Continuous-Time Gear Angle Response');
legend('Reference', 'Gear Angle', 'Location', 'best');
grid on;

% Figure 2: Motor voltage
figure(2);
plot(Vcmd.Time, Vcmd.Data, 'b', 'LineWidth', 1.5);
hold on;
yline(6, 'r--', '+6 V');
yline(-6, 'r--', '-6 V');
hold off;

xlabel('Time (s)');
ylabel('Motor Voltage (V)');
title('Continuous-Time Controller Output');
grid on;
