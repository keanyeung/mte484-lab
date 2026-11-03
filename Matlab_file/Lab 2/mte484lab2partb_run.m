% Run setup
run('mte484lab2partbsetup.m');

% Run simulation
out = sim('mte484lab2partb', 'StopTime', num2str(simTime));

% Save results
save('lab2b_data.mat', 'out', 'C1', 'D1', 'Ts', 'K1', 'tau');