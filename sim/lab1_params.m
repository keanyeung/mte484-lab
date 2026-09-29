% lab1_params.m - parameters for the Lab 1 (f) saturator simulation
%
% Run this before simulating the Simulink model so every block reads a
% variable instead of a hard-coded number:
%     >> run('lab1_params.m')
%
% Build instructions for the model: task briefs/partf_simulink_guide.md

clear; clc;

%% Plant, identified in part (e)
% theta(s)/V(s) = K1 / (s (tau s + 1)), from %OS and Tp of 30 closed-loop
% steps at Kp = -15, -20 and -25 V/rad.
% Measured: K1 = -1.883 +- 0.118 rad/(V s), tau = 0.0216 +- 0.0013 s.
% K1 is negative because +V turns the large gear clockwise, which makes theta
% decrease. The lab manual allows using K1 and Kp as positive numbers in the
% simulation: flipping both leaves the loop gain Kp*K1 unchanged, so the
% closed-loop response is identical.
K1  = 1.883;      % rad/(V s), magnitude of the measured value
tau = 0.0216;     % s

plant_num = K1;          % Transfer Fcn numerator
plant_den = [tau 1 0];   % s (tau s + 1) = tau s^2 + s

%% Controller and saturator
Kp      = 3.5;    % V/rad, magnitude; low enough to keep |V| under 6 V
ref_sat = 0.7;    % rad, saturator limits (Figure 7(b))

%% Square wave reference
ref_amp     = 1.0;                % rad, must exceed ref_sat so the saturator clips
half_period = 2.0;                % s
ref_freq_hz = 1/(2*half_period);  % Hz, for the Signal Generator block
sim_time    = 12;                 % s, three full periods

%% Predicted behaviour, for checking the simulation against
wn     = sqrt(Kp*K1/tau);        % rad/s
zeta   = 1/(2*tau*wn);           % > 1 here, so the response is overdamped
poles  = roots([tau 1 Kp*K1]);
v_peak = Kp*2*ref_sat;           % V at a full saturated step (no stiction in the model)

if zeta >= 1
    damping = 'overdamped, no overshoot';
else
    damping = 'underdamped, expect overshoot';
end

fprintf('Closed loop: wn = %.1f rad/s, zeta = %.2f (%s)\n', wn, zeta, damping);
fprintf('Poles: %.1f and %.1f 1/s\n', poles(1), poles(2));
fprintf('Peak motor voltage at a full step: %.2f V (limit 6 V)\n', v_peak);
fprintf('Reference: +-%.1f rad at %.3f Hz, clipped to +-%.1f rad\n', ...
        ref_amp, ref_freq_hz, ref_sat);
