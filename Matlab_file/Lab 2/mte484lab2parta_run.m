% Run setup
run('mte484lab2asetup.m');

% Run simulation
out = sim('mte484lab2parta', 'StopTime', num2str(simTime));

% Save results
save('lab2a_data.mat', 'out', 'K1', 'tau', 'C1');

