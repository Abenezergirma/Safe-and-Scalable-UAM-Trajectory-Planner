function [optimizedValues, allValues] = valueOptimized(distanceToPeaks, rewardLimits,discountFactor, rewards)

propagationTable = distanceToPeaks < rewardLimits; %a boolean table 

minCycle = 2; % because its a grid world fundamentally 

peakValues = rewards ./ (1.0 - discountFactor.^(minCycle));

allValues = propagationTable .* (peakValues .* discountFactor.^distanceToPeaks); %this is the key to all 

optimizedValues = max(allValues, [], 2);


end 

