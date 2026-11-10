% MTE 484 - LAB 3 PART (b)
% Continuous vs Discrete Outer-Loop Controller
% Group 125

clc;
close all;

% 1. LOAD PARAMETERS

run('mte484lab3partbsetup.m');

model = 'mte484lab3partb';
simTime = 60;

% 2. RUN SIMULATION

out = sim(model, 'StopTime', num2str(simTime));

% 3. EXTRACT SIMULATION DATA

% Reference ball position
ref = out.theta_ref1;

% Continuous system
yc = out.y_const;
thetaC = out.theta_const;
voltageC = out.Vcmd_const;

% Discrete system
yd = out.y_const_disc;
thetaD = out.theta_disc;
voltageD = out.Vcmd_disc;

% Time and position vectors
tRef = ref.Time(:);
r = ref.Data(:);

tc = yc.Time(:);
yContinuous = yc.Data(:);

td = yd.Time(:);
yDiscrete = yd.Data(:);

% 4. PLOT BALL POSITION RESPONSES

figure('Name', 'Lab 3 - Controller Comparison');

plot(tRef, r, 'k--', 'LineWidth', 1.5);
hold on;

plot(tc, yContinuous, 'b', 'LineWidth', 1.5);
plot(td, yDiscrete, 'r--', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Ball Position (m)');
title('Lab 3 - Continuous vs Discrete Ball Position');

legend('Reference', 'Continuous', 'Discrete', ...
       'Location', 'best');

grid on;
xlim([0 simTime]);

% 5. CALCULATE STEP RESPONSE METRICS

continuousResults = calculateStepMetrics( ...
    tRef, r, tc, yContinuous, simTime);

discreteResults = calculateStepMetrics( ...
    tRef, r, td, yDiscrete, simTime);

disp(' ');
disp('======================================');
disp(' CONTINUOUS CONTROLLER PERFORMANCE');
disp('======================================');
disp(continuousResults);

disp(' ');
disp('======================================');
disp(' DISCRETE CONTROLLER PERFORMANCE');
disp('======================================');
disp(discreteResults);

% 6. COMPARE CONTINUOUS VS DISCRETE

% Remove duplicate timestamps before interpolation
[tdUnique, idx] = unique(td, 'last');
ydUnique = yDiscrete(idx);

% Interpolate discrete output onto continuous time vector
ydInterpolated = interp1( ...
    tdUnique, ydUnique, tc, 'linear', 'extrap');

% Compare only over overlapping time interval
valid = (tc >= min(tdUnique)) & ...
        (tc <= max(tdUnique));

difference = yContinuous(valid) - ydInterpolated(valid);

maxDifference = max(abs(difference));
rmsDifference = sqrt(mean(difference.^2));

fprintf('\n======================================\n');
fprintf(' CONTINUOUS VS DISCRETE DIFFERENCE\n');
fprintf('======================================\n');

fprintf('Maximum position difference: %.6f m\n', ...
        maxDifference);

fprintf('Maximum position difference: %.3f mm\n', ...
        maxDifference*1000);

fprintf('RMS position difference: %.3f mm\n', ...
        rmsDifference*1000);

% 7. MOTOR VOLTAGE AND GEAR ANGLE

peakVoltageC = max(abs(voltageC.Data(:)));
peakVoltageD = max(abs(voltageD.Data(:)));

peakThetaC = max(abs(thetaC.Data(:)));
peakThetaD = max(abs(thetaD.Data(:)));

fprintf('\n======================================\n');
fprintf(' MOTOR VOLTAGE AND GEAR ANGLE\n');
fprintf('======================================\n');

fprintf('Continuous peak voltage: %.4f V\n', ...
        peakVoltageC);

fprintf('Discrete peak voltage: %.4f V\n', ...
        peakVoltageD);

fprintf('Continuous peak gear angle: %.4f rad\n', ...
        peakThetaC);

fprintf('Discrete peak gear angle: %.4f rad\n', ...
        peakThetaD);

% 8. PLOT CONTINUOUS-DISCRETE ERROR

figure('Name', 'Continuous vs Discrete Error');

plot(tc(valid), difference*1000, ...
     'm', 'LineWidth', 1.2);

xlabel('Time (s)');
ylabel('Position Difference (mm)');
title('Continuous - Discrete Ball Position Difference');

grid on;
xlim([0 simTime]);

% 9. SUMMARY

fprintf('\n======================================\n');
fprintf(' LAB 3 PART (b) SIMULATION COMPLETE\n');
fprintf('======================================\n');

fprintf('Sampling period: %.4f s\n', Ts);
fprintf('Simulation duration: %.1f s\n', simTime);

fprintf('\nPerformance requirements:\n');
fprintf('Settling time (2%%): <= 7 s\n');
fprintf('Overshoot: < 45%%\n');
fprintf('Steady-state error: 0 m\n');

% LOCAL FUNCTION
% Keep this function at the END of the script.

function results = calculateStepMetrics(tRef, ref, t, y, simTime)

    % Remove duplicate reference timestamps
    [tRef, idx] = unique(tRef, 'last');
    ref = ref(idx);

    % Detect reference changes
    stepIdx = find(abs(diff(ref)) > 0.075) + 1;

    % Ignore the incomplete transition at the simulation end
    stepIdx = stepIdx(tRef(stepIdx) < simTime - 1e-9);

    n = numel(stepIdx);

    stepTime = nan(n,1);
    fromPos = nan(n,1);
    toPos = nan(n,1);
    settlingTime = nan(n,1);
    overshoot = nan(n,1);
    endError = nan(n,1);

    for i = 1:n

        t0 = tRef(stepIdx(i));

        if i < n
            tEnd = tRef(stepIdx(i+1));
        else
            tEnd = simTime;
        end

        initialRef = ref(stepIdx(i)-1);
        target = ref(stepIdx(i));
        delta = target - initialRef;

        % Extract response for this reference interval
        mask = (t >= t0) & (t < tEnd);

        ts = t(mask);
        ys = y(mask);

        if numel(ys) < 2
            continue;
        end

        % Tracking error over final second
        finalMask = ts >= (tEnd - 1);

        if ~any(finalMask)
            finalMask(end) = true;
        end

        yFinal = mean(ys(finalMask));
        endError(i) = (target-yFinal)*1000;

        % Percentage overshoot
        if delta > 0
            overshoot(i) = max(0, max(ys)-target) ...
                           /abs(delta)*100;
        else
            overshoot(i) = max(0, target-min(ys)) ...
                           /abs(delta)*100;
        end

        % 2% settling time
        tolerance = 0.02*abs(delta);

        outside = find(abs(ys-target) > tolerance);

        if isempty(outside)
            settlingTime(i) = 0;

        elseif outside(end) < numel(ys)
            settlingTime(i) = ...
                ts(outside(end)+1)-t0;

        else
            settlingTime(i) = NaN;
        end

        stepTime(i) = t0;
        fromPos(i) = initialRef;
        toPos(i) = target;
    end

    % Results table
    results = table( ...
        stepTime, fromPos, toPos, ...
        settlingTime, overshoot, endError, ...
        'VariableNames', ...
        {'StepTime_s', 'From_m', 'To_m', ...
         'SettlingTime_s', 'Overshoot_pct', ...
         'Last1sError_mm'});
end
