clear 


%Pick the aircraft type from here 
aircraftType = struct('default', {'fwdProjectFast'}, 'simplified', {'simplified'}, 'fixedwing', {'fixedwing'}, 'multirotor',{'multirotor'});

initialStates = [100.97398, 100.69597, 100.16267, 0.00000, 0.00000, 4.74448, 0.00000,  24.01000];

model = DryVR(0.01,500); % [x, y, z] pos of the vehicle 
tic
Traces = GenerateTraces(model, aircraftType.simplified, initialStates);

initialRadii = computeInitialRadii(model,Traces);
% muPoints = computeMuPoints(model, Traces, initialRadii);
discrepancyParameters = computeDiscrepancyParameters(model, Traces, initialRadii);
reachTube = getReachtube(model, Traces, initialRadii, discrepancyParameters);
toc

save('output')
