function [totalValues] = valueFunction(ownship, states, positivePeaks, negativePeaks)
% determine the value based on position 
parameters;

numFuturePoints = length(states(:,1,1));
lenFutureTraj = length(states(1,:,1));
numStates = length(states(:,:,1));


if ~isempty(positivePeaks)
    peakLocations = positivePeaks(:,[LOCX, LOCY, LOCZ]);
    rewardLimits = positivePeaks(:,LIM);
    discountFactor = positivePeaks(:, DISC);
    rewards = positivePeaks(:, RWD);
    A = states(:,:,1:3);

    reshapedStates = reshape(permute(A,[2 1 3]),[],size(A,3),1);
    positiveDistance = pdist2(reshapedStates, peakLocations(:,1:3));%computes pairwise distance b/n states and peaks
    [positiveValues,~] = valueOptimized(positiveDistance, rewardLimits, discountFactor, rewards);


else
    positiveValues = zeros(length(states));
end

if ~isempty(negativePeaks)
    peakLocations = negativePeaks(:,[LOCX, LOCY, LOCZ]);
    rewardLimits = negativePeaks(:,LIM)'; %transpose for value optimization
    discountFactor = negativePeaks(:, DISC)';
    rewards = negativePeaks(:, RWD)';
    B = states(:,:,1:3);

    reshapedStates = reshape(permute(B,[2 1 3]),[],size(B,3),1);
    negativeDistance = pdist2(reshapedStates, peakLocations(:,1:3));%
    %     negativeDistance = pdist2(reshape(states(:,:,1:3),[],size(states(:,:,1:3),3),1), peakLocations(:,1:3)); %change states to (:,:,1:3)
    [negativeValues, ~] = valueOptimized(negativeDistance, rewardLimits, discountFactor, rewards);

    totalValues = positiveValues - negativeValues;

else
    totalValues = positiveValues;
end

if true
    altitude = states(:,:,3)';
    altitude = altitude(:);
    belowLevel = altitude < 100;
    penalityFunc = 10000 - altitude;
    totalValues(belowLevel) = totalValues(belowLevel) - penalityFunc(belowLevel);

end


totalValues = reshape(totalValues, lenFutureTraj, numFuturePoints)';

% totalValues = permute(reshape(totalValues, numFuturePoints, lenFutureTraj, 1), [1, 2, 3]);

% A = reshape(totalValues, numFuturePoints, lenFutureTraj, 1);
% 
% A == totalValues(floor(linspace(1,13500,10)),:)
% 
% linspace(1,13500,10)

end 