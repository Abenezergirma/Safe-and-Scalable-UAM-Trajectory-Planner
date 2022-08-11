function [futureStates, oneStepStates, futureActions] = neighboringStates(ownship, actions, limits)
% A function that returns next one step and few future states of the
% aircraft
% Input: 
% Output: 
% Usage: 

if ownship.hit == true
    actions(:,3) = 0;
    actions = unique(actions, 'rows');
end


futureTraj = fwdProjectFast(ownship, actions, limits, 0.1, 100);


oneStepStates = futureTraj(11,:,:); %eventually put an assertion block to check the size 

futIndex = floor(linspace(1,length(futureTraj(:,1,1)),10));

futureStates = futureTraj(futIndex,:,:); %eventually put an assertion block to check the size 

%reshaping these two arrays for next operations
futureActions = permute(repmat(actions,1,1,10), [3 1 2]) ;

oneStepStates = permute(repmat(oneStepStates,10,1,1), [1 2 3]);

end


