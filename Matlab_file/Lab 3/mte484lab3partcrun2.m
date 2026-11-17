% MTE 484 - Lab 3 Part (c)
% Improved Controller Simulation and Analysis
% Used this with the mte484lab3partc2 to be able to test and find the soln

clearvars;
close all;
clc;

% Initialize complete Part (c) setup
run('mte484lab3partcsetup.m');

% Run Simulink
model = 'mte484lab3partc2';
out = sim(model, 'StopTime', num2str(simTime));

% Extract signals
ref = out.theta_ref1;

yc = out.y_const;
yd = out.y_const_disc;

thetaC = out.theta_const;
thetaD = out.theta_disc;

voltageC = out.Vcmd_const;
voltageD = out.Vcmd_disc;

% Extract reference
tRefRaw = ref.Time(:);
rRaw = ref.Data(:);

% Extract ball positions
tc = yc.Time(:);
yContinuous = yc.Data(:);

td = yd.Time(:);
yDiscrete = yd.Data(:);

% Plot continuous and discrete response
figure('Name', 'Lab 3 Part (c) - Ball Position');

plot(tRefRaw, rRaw, 'k--', 'LineWidth', 1.5);
hold on;
plot(tc, yContinuous, 'b', 'LineWidth', 1.5);
plot(td, yDiscrete, 'r--', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Ball Position (m)');
title('Lab 3 Part (c) - Improved Controller');

legend('Reference', 'Continuous', 'Discrete', ...
    'Location', 'best');

grid on;
xlim([0 simTime]);

% Remove duplicate reference timestamps
[tRef, idxRef] = unique(tRefRaw, 'last');
r = rRaw(idxRef);

% Detect reference transitions
stepIdx = find(abs(diff(r)) > 0.075) + 1;

% Exclude incomplete final transition
stepIdx = stepIdx(tRef(stepIdx) < simTime - 1e-9);

n = numel(stepIdx);

% Evaluate both controllers
modelNames = {'Continuous', 'Discrete'};
timeData = {tc, td};
positionData = {yContinuous, yDiscrete};

results = cell(1,2);

for m = 1:2

    t = timeData{m};
    y = positionData{m};

    stepTime = nan(n,1);
    fromPos = nan(n,1);
    toPos = nan(n,1);
    settlingTime = nan(n,1);
    overshoot = nan(n,1);
    last1sError = nan(n,1);

    for i = 1:n

        t0 = tRef(stepIdx(i));

        if i < n
            tEnd = tRef(stepIdx(i+1));
        else
            tEnd = simTime;
        end

        initialRef = r(stepIdx(i)-1);
        target = r(stepIdx(i));
        delta = target-initialRef;

        mask = (t >= t0) & (t < tEnd);

        ts = t(mask);
        ys = y(mask);

        if numel(ys) < 2
            continue;
        end

        % Last-second tracking error
        finalMask = ts >= (tEnd-1);

        if ~any(finalMask)
            finalMask(end) = true;
        end

        yFinal = mean(ys(finalMask));
        last1sError(i) = (target-yFinal)*1000;

        % Percentage overshoot
        if delta > 0
            overshoot(i) = ...
                max(0,max(ys)-target)/abs(delta)*100;
        else
            overshoot(i) = ...
                max(0,target-min(ys))/abs(delta)*100;
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

    results{m} = table( ...
        stepTime, fromPos, toPos, ...
        settlingTime, overshoot, last1sError, ...
        'VariableNames', ...
        {'StepTime_s','From_m','To_m', ...
         'SettlingTime_s','Overshoot_pct', ...
         'Last1sError_mm'});

    fprintf('\n=================================\n');
    fprintf('%s PERFORMANCE\n', upper(modelNames{m}));
    fprintf('=================================\n');

    disp(results{m});

    % Check worst-case performance
    st = settlingTime;
    os = overshoot;

    if any(~isfinite(st))
        worstST = NaN;
        settlingPass = false;
    else
        worstST = max(st);
        settlingPass = worstST <= 7;
    end

    worstOS = max(os);
    overshootPass = all(isfinite(os)) && worstOS < 45;

    fprintf('Worst settling time: %.4f s\n',worstST);
    fprintf('Worst overshoot: %.4f %%\n',worstOS);

    fprintf('Settling requirement met: %d\n', ...
        settlingPass);

    fprintf('Overshoot requirement met: %d\n', ...
        overshootPass);
end

% Save results tables
resultsContinuous = results{1};
resultsDiscrete = results{2};

% Compare continuous and discrete positions
[tdUnique, idxD] = unique(td, 'last');

ydUnique = yDiscrete(idxD);

ydInterp = interp1( ...
    tdUnique, ydUnique, tc, 'linear', 'extrap');

valid = (tc >= min(tdUnique)) & ...
        (tc <= max(tdUnique));

difference = yContinuous(valid)-ydInterp(valid);

maxDifference = max(abs(difference));
rmsDifference = sqrt(mean(difference.^2));

fprintf('\n=================================\n');
fprintf('CONTINUOUS VS DISCRETE DIFFERENCE\n');
fprintf('=================================\n');

fprintf('Maximum difference: %.4f mm\n', ...
    maxDifference*1000);

fprintf('RMS difference: %.4f mm\n', ...
    rmsDifference*1000);

% Plot difference
figure('Name','Part (c) - Discretization Error');

plot(tc(valid),difference*1000, ...
    'm','LineWidth',1.3);

xlabel('Time (s)');
ylabel('Position Difference (mm)');
title('Part (c) - Continuous Minus Discrete');

grid on;
xlim([0 simTime]);

% Peak motor voltage and gear angle
peakVoltageC = max(abs(voltageC.Data(:)));
peakVoltageD = max(abs(voltageD.Data(:)));

peakThetaC = max(abs(thetaC.Data(:)));
peakThetaD = max(abs(thetaD.Data(:)));

fprintf('\n=================================\n');
fprintf('MOTOR VOLTAGE AND GEAR ANGLE\n');
fprintf('=================================\n');

fprintf('Continuous peak voltage: %.4f V\n', ...
    peakVoltageC);

fprintf('Discrete peak voltage: %.4f V\n', ...
    peakVoltageD);

fprintf('Continuous peak gear angle: %.4f rad\n', ...
    peakThetaC);

fprintf('Discrete peak gear angle: %.4f rad\n', ...
    peakThetaD);

fprintf('\n=================================\n');
fprintf('LAB 3 PART (c) COMPLETE\n');
fprintf('=================================\n');

fprintf('Sampling period: %.4f s\n',Ts);
fprintf('Simulation duration: %.1f s\n',simTime);
fprintf('Nominal DC tracking error: %.6g m/m\n', ...
    1-dcgain(Tfull));

fprintf('\nPerformance targets:\n');
fprintf('2%% settling time <= 7 s\n');
fprintf('Overshoot < 45%%\n');
fprintf('Asymptotic steady-state error = 0 m\n');


% Check initial transition (0 to 10 seconds)

target = 0.25;
initial = 0.10;
delta = target - initial;
tolerance = 0.02*delta;

for m = 1:2

    if m == 1
        tCheck = tc;
        yCheck = yContinuous;
        name = 'Continuous';
    else
        tCheck = td;
        yCheck = yDiscrete;
        name = 'Discrete';
    end

    mask = tCheck >= 0 & tCheck < 10;
    tt = tCheck(mask);
    yy = yCheck(mask);

    peak = max(yy);
    OS = max(0, peak-target)/delta*100;

    outside = find(abs(yy-target) > tolerance);

    if isempty(outside)
        settling = 0;
    elseif outside(end) < numel(tt)
        settling = tt(outside(end)+1);
    else
        settling = NaN;
    end

    fprintf('\n%s initial response:\n', name);
    fprintf('Initial position: %.4f m\n', yy(1));
    fprintf('Peak position: %.4f m\n', peak);
    fprintf('Overshoot: %.3f %%\n', OS);
    fprintf('Settling time: %.3f s\n', settling);
    fprintf('Meets settling requirement: %d\n', ...
        isfinite(settling) && settling <= 7);
    fprintf('Meets overshoot requirement: %d\n', ...
        OS < 45);

end



% Export Lab 3 Part (c2) simulation results to CSV

outputFolder = 'csv_exports';

if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

% Create common time vector (1 ms sampling)
timeCSV = (0:Ts:simTime)';

% Extract and interpolate simulation signals
referenceCSV = interp1(tRefRaw, rRaw, timeCSV, ...
    'previous', 'extrap');

yContinuousCSV = interp1(tc, yContinuous, timeCSV, ...
    'linear', 'extrap');

yDiscreteCSV = interp1(td, yDiscrete, timeCSV, ...
    'previous', 'extrap');

% Extract motor voltage and gear angle
voltageContinuousCSV = interp1( ...
    voltageC.Time, voltageC.Data(:), timeCSV, ...
    'linear', 'extrap');

voltageDiscreteCSV = interp1( ...
    voltageD.Time, voltageD.Data(:), timeCSV, ...
    'previous', 'extrap');

thetaContinuousCSV = interp1( ...
    thetaC.Time, thetaC.Data(:), timeCSV, ...
    'linear', 'extrap');

thetaDiscreteCSV = interp1( ...
    thetaD.Time, thetaD.Data(:), timeCSV, ...
    'linear', 'extrap');

% Calculate position difference
positionDifferenceCSV = ...
    yContinuousCSV - yDiscreteCSV;

% Build time-series table
simulationData = table( ...
    timeCSV, referenceCSV, ...
    yContinuousCSV, yDiscreteCSV, ...
    thetaContinuousCSV, thetaDiscreteCSV, ...
    voltageContinuousCSV, voltageDiscreteCSV, ...
    positionDifferenceCSV, ...
    'VariableNames', { ...
    'Time_s', 'Reference_m', ...
    'Y_Continuous_m', 'Y_Discrete_m', ...
    'Theta_Continuous_rad', 'Theta_Discrete_rad', ...
    'Voltage_Continuous_V', 'Voltage_Discrete_V', ...
    'Position_Difference_m'});

% Export full simulation time series
writetable(simulationData, ...
    fullfile(outputFolder, 'lab3_partc2_timeseries.csv'));

% Export step performance results
writetable(resultsContinuous, ...
    fullfile(outputFolder, 'lab3_partc2_continuous_performance.csv'));

writetable(resultsDiscrete, ...
    fullfile(outputFolder, 'lab3_partc2_discrete_performance.csv'));

% Export summary
metric = { ...
    'Maximum_Position_Difference_mm';
    'RMS_Position_Difference_mm';
    'Peak_Continuous_Voltage_V';
    'Peak_Discrete_Voltage_V';
    'Peak_Continuous_Gear_Angle_rad';
    'Peak_Discrete_Gear_Angle_rad';
    'Sampling_Period_s'};

value = [ ...
    maxDifference*1000;
    rmsDifference*1000;
    peakVoltageC;
    peakVoltageD;
    peakThetaC;
    peakThetaD;
    Ts];

summaryData = table(metric, value, ...
    'VariableNames', {'Metric', 'Value'});

writetable(summaryData, ...
    fullfile(outputFolder, 'lab3_partc2_summary.csv'));

fprintf('\nCSV files successfully generated in: %s\n', ...
    fullfile(pwd, outputFolder));
