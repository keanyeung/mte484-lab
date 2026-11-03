% MTE 484 - Lab 2 Part (b)
% Continuous vs Discrete Controller Comparison

% Load simulation results
load('lab2b_data.mat');

% Extract simulation data
ref = out.theta_ref_sat1;

theta_cont = out.theta_const;
theta_disc = out.theta_disc;

Vcmd_cont = out.Vcmd_const;
Vcmd_disc = out.Vcmd_disc;


% Figure 1: Gear angle response
figure(1);

plot(ref.Time, ref.Data, 'k--', 'LineWidth', 1.5);
hold on;
plot(theta_cont.Time, theta_cont.Data, 'b', 'LineWidth', 1.5);
plot(theta_disc.Time, theta_disc.Data, 'r--', 'LineWidth', 1.5);
hold off;

xlabel('Time (s)');
ylabel('Gear Angle (rad)');
title('Continuous vs Discrete Gear Angle Response');
legend('Reference', 'Continuous', 'Discrete', 'Location', 'best');
grid on;


% Figure 2: Motor voltage
figure(2);

plot(Vcmd_cont.Time, Vcmd_cont.Data, 'b', 'LineWidth', 1.5);
hold on;
plot(Vcmd_disc.Time, Vcmd_disc.Data, 'r--', 'LineWidth', 1.5);

yline(6, 'k--', '+6 V');
yline(-6, 'k--', '-6 V');
hold off;

xlabel('Time (s)');
ylabel('Motor Voltage (V)');
title('Continuous vs Discrete Motor Voltage');
legend('Continuous', 'Discrete', 'Location', 'best');
grid on;


% Calculate closed-loop performance
G = tf(K1, [tau 1 0]);

T_cont = feedback(C1 * G, 1);

G_disc = c2d(G, Ts, 'zoh');
T_disc = feedback(D1 * G_disc, 1);

info_cont = stepinfo(T_cont);
info_disc = stepinfo(T_disc);


% Display results
fprintf('\n--- Continuous Controller ---\n');
fprintf('Settling Time: %.4f s\n', info_cont.SettlingTime);
fprintf('Overshoot: %.2f %%\n', info_cont.Overshoot);
fprintf('Peak Motor Voltage: %.4f V\n', max(abs(Vcmd_cont.Data)));

fprintf('\n--- Discrete Controller ---\n');
fprintf('Settling Time: %.4f s\n', info_disc.SettlingTime);
fprintf('Overshoot: %.2f %%\n', info_disc.Overshoot);
fprintf('Peak Motor Voltage: %.4f V\n', max(abs(Vcmd_disc.Data)));