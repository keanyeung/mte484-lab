% Lab 3 Part (c) - Improved Outer Controller Setup

% Plant parameters from Labs 1 and 2
K1 = -1.883;
tau = 0.0216;
K2 = 0.0609;
K3 = -4.526;

% Inner controller design from Lab 2
a = 51;
sigma = 14;
wd = 10;

Kmag = abs(K1);

pc = a + 2*sigma - 1/tau;
k = (tau*(sigma^2 + wd^2 + 2*a*sigma) - pc)/Kmag;
zc = tau*a*(sigma^2 + wd^2)/(Kmag*k);

Cnum = -k*[1 zc];
Cden = [1 pc];

C1 = tf(Cnum, Cden);

% Sampling and simulation settings
Ts = 0.001;
simTime = 60;
sat = 0.7;

% Ball-position square wave
A = 0.15;
bias = 0.10;
period = 20;
freq = 1/period;
pulseWidth = 50;

% Discrete inner controller from Lab 2
D1 = c2d(C1, Ts, 'tustin');
[Dnum, Dden] = tfdata(D1, 'v');

% Part (c) outer controller design
% Approximate inner closed loop as unity gain
% Py(s) = K2*K3/s^2

g = -K2*K3;

% Desired approximate closed-loop poles
sigma2 = 0.9;
wd2 = 0.6;
p3 = 2;
p4 = 4;

desiredPoles = [ ...
    -sigma2+1i*wd2, ...
    -sigma2-1i*wd2, ...
    -p3, -p4];

% Desired characteristic polynomial
Pd = real(poly(desiredPoles));

% Controller structure:
% C2(s) = -(a2*s^2+a1*s+a0)/(s*(s+p))

% Coefficient matching
C2num = -Pd(3:5)/g;
C2den = [1 Pd(2) 0];

C2 = tf(C2num, C2den);

% Tustin discretization of outer controller
D2 = c2d(C2, Ts, 'tustin');

[D2num, D2den] = tfdata(D2, 'v');

% Full nominal linear model
Gm = tf(K1, [tau 1 0]);

% Actual inner closed-loop transfer function
T1 = feedback(C1*Gm, 1);

% Outer plant with complete inner-loop dynamics
Py = T1*tf(K2*K3, [1 0 0]);

% Full unsaturated outer closed loop
Tfull = feedback(C2*Py, 1);

% Verify nominal closed-loop stability
closedLoopPoles = pole(Tfull);

if any(real(closedLoopPoles) >= 0)
    warning('Nominal closed loop may be unstable.');
end

% Display results
format long g

disp('Continuous Inner Controller C1(s):');
C1

disp('Discrete Inner Controller D1(z):');
D1

disp('Desired Outer Poles:');
disp(desiredPoles.');

disp('Desired Characteristic Polynomial:');
disp(Pd);

disp('Continuous Outer Controller C2(s):');
C2

disp('Discrete Outer Controller D2(z):');
D2

disp('Discrete Outer Numerator:');
disp(D2num);

disp('Discrete Outer Denominator:');
disp(D2den);

disp('Full Nominal Closed-Loop Poles:');
disp(closedLoopPoles);

fprintf('Nominal closed-loop DC gain: %.6f\n', ...
    dcgain(Tfull));
