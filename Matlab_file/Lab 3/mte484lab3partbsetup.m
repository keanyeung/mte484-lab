% Lab 3 Part (b) - Outer Controller Discretization

Ts = 0.001;

C2 = tf([-7 -2.45], [1 2.5]);

% Tustin discretization
D2 = c2d(C2, Ts, 'tustin');

% Coefficients for Simulink and Arduino
[D2num, D2den] = tfdata(D2, 'v');

format long g

disp('Discrete Outer Controller D2(z):');
D2

disp('Numerator:');
disp(D2num);

disp('Denominator:');
disp(D2den);