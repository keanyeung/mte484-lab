
% LAB 3 PART (a) - RUN AND ANALYZE

% Ball and beam parameters
K2 = 0.0609;
K3 = -4.526;

% Outer controller
C2 = tf([-7 -2.45], [1 2.5]);

% Simulation settings
Ts = 0.001;
simTime = 60;
sat = 0.7;

% Reference settings
A = 0.15;
period = 20;
bias = 0.10;

% Run Simulink
model = 'mte484lab3parta';

out = sim(model, 'StopTime', num2str(simTime));

% Extract logged signals
% Names correspond to your current Simulink screenshot
ref = out.get('theta_ref1');       % Ball reference
ball = out.get('y_const');    % Ball position
theta = out.get('theta_const');    % Gear angle
voltage = out.get('Vcmd_const');   % Motor voltage

tRef = ref.Time(:);
yRef = squeeze(ref.Data);
yRef = yRef(:);

t = ball.Time(:);
y = squeeze(ball.Data);
y = y(:);

% Plot reference vs actual ball position
figure('Name', 'Lab 3 Ball Position');

plot(tRef, yRef, 'k--', 'LineWidth', 1.5);
hold on;
plot(t, y, 'b', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Ball Position (m)');
title('Lab 3 - Ball Position Tracking');
legend('Reference', 'Actual Position', 'Location', 'best');
grid on;
xlim([0 simTime]);

% Detect reference transitions
% Remove duplicate timestamps at pulse transitions
[tRef, idx] = unique(tRef, 'last');
yRef = yRef(idx);

stepIdx = find(abs(diff(yRef)) > 0.075) + 1;
n = numel(stepIdx);

stepTime = nan(n,1);
fromPos = nan(n,1);
toPos = nan(n,1);
settlingTime = nan(n,1);
overshoot = nan(n,1);
SSE = nan(n,1);

% Calculate performance for each transition
for i = 1:n

    t0 = tRef(stepIdx(i));

    if i < n
        tEnd = tRef(stepIdx(i+1));
    else
        tEnd = simTime;
    end

    initialRef = yRef(stepIdx(i)-1);
    target = yRef(stepIdx(i));
    delta = target - initialRef;

    mask = (t >= t0) & (t < tEnd);

    ts = t(mask);
    ys = y(mask);

    if numel(ys) < 2
        continue;
    end

    % Steady-state error: mean over final second
    finalMask = ts >= (tEnd - 1);
    yFinal = mean(ys(finalMask));
    SSE(i) = target - yFinal;

    % Percentage overshoot
    if delta > 0
        overshoot(i) = max(0, max(ys)-target) ...
                       / abs(delta) * 100;
    else
        overshoot(i) = max(0, target-min(ys)) ...
                       / abs(delta) * 100;
    end

    % 2% settling time
    tolerance = 0.02 * abs(delta);
    outside = find(abs(ys-target) > tolerance);

    if isempty(outside)
        settlingTime(i) = 0;
    elseif outside(end) < numel(ys)
        settlingTime(i) = ...
            ts(outside(end)+1) - t0;
    else
        settlingTime(i) = NaN;
    end

    stepTime(i) = t0;
    fromPos(i) = initialRef;
    toPos(i) = target;
end

% Results table
results = table(stepTime, fromPos, toPos, ...
    settlingTime, overshoot, SSE, ...
    'VariableNames', ...
    {'StepTime_s', 'From_m', 'To_m', ...
     'SettlingTime_s', 'Overshoot_pct', 'SSE_m'});

disp(' ');
disp('===== LAB 3 PERFORMANCE RESULTS =====');
disp(results);

% Additional results
peakVoltage = max(abs(voltage.Data(:)));
peakTheta = max(abs(theta.Data(:)));

fprintf('\nPeak motor voltage: %.4f V\n', peakVoltage);
fprintf('Peak gear angle: %.4f rad\n', peakTheta);

