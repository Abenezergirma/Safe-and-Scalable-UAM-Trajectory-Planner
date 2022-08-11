% function [totalNMACs,stepTimer] = main()
clear; close all; clc;
addContainingDirAndSubDir;
parameters;
teamActions = buildActions;
teamLimits = buildLimits;
scenarioSelector = struct('circle','circle', 'random','random');
% instantiating the ownship class for each aircraft in the game
totalAgents = 2;
droneList = cell(1,totalAgents);
initialStates = scenarioGenerator(totalAgents,'circle');
% goals = [10000, 16000, 700];
totalNMACs = 0;

goals = circshift(initialStates,totalAgents/2);

for i = 1: totalAgents
    aircraftID = i;
    aircraftTeam = 1; %only one team
    goal = goals(i,1:3);
    baseLatitude = 10;
    baseAltitude = 10;
    droneInitialStates = initialStates(i,:);
    drone = Ownship(aircraftID, aircraftTeam, baseLatitude, baseAltitude, goal);
    drone = updateAircraftStates(drone,  droneInitialStates);
    droneList{i} =  drone;

end
%%
numLeft = size(droneList,2);
stepTimer = [];
while numLeft > 0
    tic
    for k = 1:numel(droneList)
        % The following is the entire operation for aircraft k at step j
        ownship = droneList{k};

        %check if the aircraft is still in the game
        if ownship.dead
            continue
        end

        %build both positive and negative reward sources
        [positivePeaks, negativePeaks] = buildPeaks(ownship, droneList);

        %Forward project all the posible states of the aircraft for the next 10 secs
        [futureStates, oneStepStates, futureActions] = neighboringStates(ownship, teamActions, teamLimits);

        % compute the value for each future state
        totalValues = valueFunction(ownship, futureStates, positivePeaks, negativePeaks);
        
        %select the optimal action
        ownship = selectBestAction(ownship, totalValues, futureActions, futureStates, oneStepStates);

        % update the aircraft state based the selected action
        ownship = updateAircraftStates(ownship,  ownship.nextStates);

        % check if the ownship makes it to goal
        if norm(ownship.currentStates(1:3) - ownship.goal) < 30
            ownship.dead = true;
            disp('ownship ' + string(ownship.aircraftID) + ' made it to goal, removed') 
            numLeft = numLeft -1;
        end

        droneList{k} = ownship;

    end

    % monitor the status of the game
    [terminal, NMACs, droneList] = terminalDetection(droneList);
    totalNMACs = totalNMACs + NMACs;
    stepTimer = [stepTimer,toc];

%     disp('step '+string(j) + ', computation time is ' + string(toc) + ' sec')
    fprintf('NMACs so far, ' + string(totalNMACs) +'\n')
    fprintf('Num aircraft left, ' + string(numLeft) +'\n')

    if terminal==true
        return
    end
end

% %%
% allNMAC = totalNMACs
% Mean = mean(stepTimer)
% Median = median(stepTimer)
% Std = std(stepTimer)
% Throughput = sum(stepTimer)

% save('MDP')
% end

