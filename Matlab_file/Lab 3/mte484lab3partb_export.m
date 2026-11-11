% LAB 3 PART (b) - EXPORT SIMULATION FOR THE HARDWARE OVERLAY
% Runs mte484lab3partb.slx and writes sim_lab3b.csv on a 10 ms grid,
% the same rate as the lab3_outer_loop stream:
%   t_s, r, y_cont, y_disc, theta_disc
% analyze_lab3b.py (folder "3b outer loop readings") reads this file.

run('mte484lab3partbsetup.m');

model = 'mte484lab3partb';
simTime = 60;
out = sim(model, 'StopTime', num2str(simTime));

tGrid = (0:0.01:simTime).';

r      = onGrid(out.theta_ref1,   tGrid);
yCont  = onGrid(out.y_const,      tGrid);
yDisc  = onGrid(out.y_const_disc, tGrid);
thDisc = onGrid(out.theta_disc,   tGrid);

T = table(tGrid, r, yCont, yDisc, thDisc, ...
    'VariableNames', {'t_s', 'r', 'y_cont', 'y_disc', 'theta_disc'});

outFile = fullfile(fileparts(mfilename('fullpath')), ...
    '..', '..', '3b outer loop readings', 'sim_lab3b.csv');
writetable(T, outFile);
fprintf('Wrote %s (%d rows)\n', outFile, height(T));

% Resamples a logged timeseries onto tGrid. Reference edges log two points
% at the same time; 'last' keeps the value after the edge, 'previous'
% holds it, so the square wave stays square.
function v = onGrid(ts, tGrid)
    [t, idx] = unique(ts.Time(:), 'last');
    d = squeeze(ts.Data);
    d = d(:);
    v = interp1(t, d(idx), tGrid, 'previous', 'extrap');
end
