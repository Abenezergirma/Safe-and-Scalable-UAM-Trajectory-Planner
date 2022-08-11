classdef Ownship
    properties %Uppercase -- TODO
        
        % general Aircraft properties 
        L = 0.9; %Lift acceleration
        g = 9.81
        timestep = 0.1;
        aicraftLimits;
        aircraftActions;

        % Aircraft states
        currentStates = zeros(1,8);
        x = zeros(1,1);
        y = zeros(1,1);
        z = zeros(1,1);
        psi = zeros(1,1);
        gamma = zeros(1,1);
        phi = zeros(1,1);
        alpha_= zeros(1,1);
        xDot = zeros(1,1);
        yDot = zeros(1,1);
        zDot = zeros(1,1);
        V = zeros(1,1);
        position = zeros(1,3);
        velocity = zeros(1,3);
        nextStates = zeros(1,8);

        traveledPath; % stores the path traveled by the aircraft so far
        pastControls; % stores the control actions applied by the aircraft so far
        bestTrajectory;% stores the best trajectories picked by the aircraft so far
 
        
        %control actions
        n_x = zeros(1,1);
        controlActions;
        alphaRate;
        rollRate;

        %Vehicle identifiers 
        aircraftID;
        aircraftTeam;
        dead = false; 
        baseLatitude;
        baseAltitude; 
        goal;

        %vehicle performace parameters
        hit = false;
        hitCounter;

    end

    methods 
        function obj = Ownship(aircraftID, aircraftTeam, baseLatitude, baseAltitude, goal)
            % A method that initializes each aircraft in the game 
            obj.aircraftID = aircraftID;
            obj.aircraftTeam = aircraftTeam;
            obj.baseLatitude = baseLatitude;
            obj.baseAltitude = baseAltitude;
            obj.goal = goal; 
            obj.aicraftLimits = buildLimits;
            obj.aircraftActions = buildActions;
        
        end

        function [obj] = updateAircraftStates(obj,  currentStates)
            % A method that updates the states of the aircraft 
            obj.x = currentStates(1);
            obj.y = currentStates(2);
            obj.z = currentStates(3);
            obj.psi = currentStates(4);
            obj.gamma = currentStates(5);
            obj.alpha_ = currentStates(6);
            obj.phi = currentStates(7);
            obj.V = currentStates(8);
            obj.position = [obj.x, obj.y, obj.z];
            obj.currentStates = currentStates; 

        end

        function [obj] = updateControlActions(obj,  bestActions)
            % A method that updates the control actions based on the
            % selected best actions 
            obj.rollRate = bestActions(1);
            obj.alphaRate= bestActions(2);
            obj.n_x = bestActions(3);
            obj.controlActions = bestActions; 
        end

        function [obj] = selectBestAction(obj, totalValues, futureActions, futureStates, oneStepStates) 
        % A method that selects the optimal action
        bestValue = max(totalValues, [],'all');
        [bestRow, bestColumn, ~] = find(totalValues==bestValue); 
        randomIndexRow = abs(randi(length(bestRow))); % randomly pick one index with max value
        randomIndexCol = abs(randi(length(bestColumn))); 
        bestActions = futureActions(bestRow(randomIndexRow), bestColumn(randomIndexCol),:);
        
        % select the best next state
        bestStep = oneStepStates(bestRow(randomIndexRow), bestColumn(randomIndexCol),:);

        %select the best future trajectory for the next 10 sec
        bestTraj = squeeze(futureStates(:, bestColumn(randomIndexCol),1:3)) ;

        % update the states and control actions of the aircraft
        obj = obj.updateControlActions( bestActions);
        obj.nextStates = squeeze(bestStep)';
        
        %store the control actions and path traveled so far
        obj.traveledPath = [obj.traveledPath; obj.nextStates]; 
        obj.pastControls = [obj.pastControls; obj.controlActions];
        obj.bestTrajectory = [obj.bestTrajectory; bestTraj];

    end

    end
end