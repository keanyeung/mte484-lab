% Plant parameters from Lab 1
K1 = -1.883;
tau = 0.0216;

% Controller design
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

% Saturation
sat = 0.7;

% Reference square wave
A = 1.0;
freq = 0.5;

% Simulation
simTime = 6;
Ts = 0.001;
