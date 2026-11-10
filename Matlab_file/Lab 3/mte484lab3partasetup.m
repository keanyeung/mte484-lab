% LAB 3 PART (a) - OUTER LOOP DESIGN

% Lab 3 - Ball and beam parameters
K2 = 0.0609;
K3 = -4.526;

% Ball dynamics
Gball = tf(K3, [1 0 0]);

% Combined gear-angle to ball-position dynamics
Gtheta_ball = tf(K2*K3, [1 0 0]);

% Outer-loop lead controller
C2 = tf(-7*[1 0.35], [1 2.5]);
C2num = [-7 -2.45]; 
C2dem = [1 2.5]; 

% Simulation parameters
Ts = 0.001;
simTime = 60;

% Gear-angle saturation
sat = 0.7;

% Ball-position reference
yLow = 0.10;       % m
yHigh = 0.25;      % m
A = yHigh - yLow;  % 0.15 m
bias = yLow;       % 0.10 m
period = 20;       % seconds

% Discretize the outer-loop controller
D2 = c2d(C2, Ts, 'tustin');

% Display controllers
disp('Outer Continuous Controller:');
C2

disp('Outer Discrete Controller:');
D2