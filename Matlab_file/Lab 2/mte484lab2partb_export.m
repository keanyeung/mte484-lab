% MTE 484 - Lab 2 Part (b) -> Part (c)
% Exports the discrete-loop step responses from mte484lab2partb.slx as a CSV
% that analyze_lab2c.py overlays on the hardware captures.
%
% Output: ../../2c inner loop readings/sim_lab2b_steps.csv
%   dir,t_ms,theta,V      one up step (-0.7 -> +0.7) and one down step,
%                         800 samples at 1 ms, t_ms = 0 at the reference edge
%
% Run from this folder:  >> mte484lab2partb_export

run('mte484lab2partbsetup.m');

% Finer solver steps so the short voltage peak is not missed (default 1 ms)
out = sim('mte484lab2partb', 'StopTime', '6', 'MaxStep', '1e-4');

ref   = out.theta_ref_sat1;
theta = out.theta_disc;
Vcmd  = out.Vcmd_disc;

nSamples = 800;          % must match CAPTURE_MS in lab2_inner_loop.ino
tk = (0:nSamples-1)' * Ts;

% Use the edges at t ~ 3 s (up) and t ~ 4 s (down): both start from a settled level
[tr, rr] = uniqueSeries(ref);
tUp   = tr(find(tr > 2.5 & rr > 0, 1));
tDown = tr(find(tr > 3.5 & rr < 0, 1));

rows = [exportStep(+1, tUp,   tk, theta, Vcmd);
        exportStep(-1, tDown, tk, theta, Vcmd)];

outFile = fullfile('..', '..', '2c inner loop readings', 'sim_lab2b_steps.csv');
fid = fopen(outFile, 'w');
if fid < 0
    error('could not open %s for writing', outFile);
end
fprintf(fid, 'dir,t_ms,theta,V\n');
for i = 1:size(rows, 1)
    if rows(i, 1) > 0, d = '+'; else, d = '-'; end
    fprintf(fid, '%s,%d,%.6f,%.6f\n', d, rows(i, 2), rows(i, 3), rows(i, 4));
end
fclose(fid);
fprintf('wrote %d rows to %s\n', size(rows, 1), outFile);
fprintf('up edge at %.4f s, down edge at %.4f s\n', tUp, tDown);


function rows = exportStep(direction, tEdge, tk, theta, Vcmd)
    % theta is continuous: interpolate at each sample instant.
    % V is held by the ZOH: take the value applied from each sample instant on.
    [tt, yt] = uniqueSeries(theta);
    [tv, yv] = uniqueSeries(Vcmd);
    tq = tEdge + tk;
    th = interp1(tt, yt, tq, 'linear');
    v  = interp1(tv, yv, tq + 1e-9, 'previous');
    rows = [repmat(direction, numel(tk), 1), round(tk * 1000), th, v];
end

function [t, y] = uniqueSeries(ts)
    % Simulink logs two points at a discontinuity; keep the later one
    [t, ia] = unique(ts.Time(:), 'last');
    y = ts.Data(:);
    y = y(ia);
end
