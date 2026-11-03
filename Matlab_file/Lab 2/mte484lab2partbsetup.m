
% Lab 2 Part (b) - Discrete Controller Setup

% Load continuous-time controller from Part (a)
run('mte484lab2asetup.m');

% Discretization
Ts = 0.001;                 % Sampling period [s]
D1 = c2d(C1, Ts, 'tustin');

% Extract discrete transfer function coefficients
[Dnum, Dden] = tfdata(D1, 'v');

% Display controller
format long g
disp('Continuous-Time Controller C1(s):');
C1

disp('Discrete-Time Controller D1(z):');
D1

disp('Discrete Numerator:');
disp(Dnum);

disp('Discrete Denominator:');
disp(Dden);
