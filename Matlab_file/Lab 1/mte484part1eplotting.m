% MTE 484 Lab 1 - Part 1e Plotting

% Extract simulation data
t_ref = out.theta_ref.Time;
theta_ref = out.theta_ref.Data;

t_sat = out.theta_ref_sat.Time;
theta_ref_sat = out.theta_ref_sat.Data;

t_theta = out.theta.Time;
theta = out.theta.Data;


% Figure 6 - Unsaturated and Saturated Reference Signals
figure(6);
clf;

plot(t_ref, theta_ref, 'LineWidth', 1.5);
hold on;
plot(t_sat, theta_ref_sat, 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Reference Angle (rad)');
title('Unsaturated and Saturated Reference Signals');

legend('Unsaturated Reference', ...
       'Saturated Reference', ...
       'Location', 'best');

ylim([-1.2 1.2]);


% Figure 7 - Simulated Plant Response
figure(7);
clf;

plot(t_sat, theta_ref_sat, '--', 'LineWidth', 1.5);
hold on;
plot(t_theta, theta, 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Angular Position (rad)');
title('Simulated Plant Response with K_p = 3.5 V/rad');

legend('Saturated Reference', ...
       'Simulated Response', ...
       'Location', 'best');

ylim([-1 1]);