classdef reachabilityAnalysis
    properties
        L = 0.9;  % Lift acceleration 
        g = 9.81; % gravity
        Tf = 10;

        gammaMax = deg2rad(20);
        gammaMin = deg2rad(-20);
        psiMax = deg2rad(20);
        psiMin = deg2rad(-20);
        VMin = 24.1789; %m/s
        VMax = 68.4211;

        n_x = 3; % assign this value from outside
        alpha_ = 0.2;  % assign this value from outside
        phi = 0.2;  % assign this value from outside

        numTraces = 20;
        x;
        y;
        z;
        
        % parameters for discrepancy calculation
        EPSILON = 1.0e-100;
        trueMIN = -10;
         
    end

    methods 
        function obj = reachabilityAnalysis(val)
            obj.x = val(1);
            obj.y = val(2);
            obj.z = val(3);
        end

        function derivative = uavDynamics(obj, y)

            [~,~,~, ~, gamma, V] =  deal(y(1), y(2), y(3), y(4), y(5), y(6) );

            n_f = obj.n_x * sin( obj.alpha_) + obj.L; %sin and cos work in rad by default 

            V_dot = obj.g * (obj.n_x * cos(obj.alpha_) - sin(gamma));

            gamma_dot = (obj.g / V) * (n_f * cos(obj.phi) - cos(gamma));

            psi_dot = obj.g * (n_f * sin(obj.phi) / (V * cos(gamma)));

            x_dot = V * cos(gamma) * cos(obj.phi);

            y_dot = V * cos(gamma) * sin(obj.phi);

            z_dot = V * sin(gamma);

            derivative = [x_dot; y_dot; z_dot; psi_dot; gamma_dot; V_dot];

        end


        function Traces = GenerateTraces(obj, varargin) %Edit this with varargin so it can accept different models

            % Set defaults
            vehicle = 'simplified';

            % Process optional inputs
            arg = varargin;
            if ischar(arg{1})
                vehicle = arg{1};
            end

            switch lower(vehicle)
                case {'simplified'}


                    initialUpper = [obj.x+5, obj.y+5, obj.z+5, obj.psiMax, obj.gammaMax, obj.VMax]; %during execution assign x,y,z from MDP
                    initialLower = [obj.x-5, obj.y-5, obj.z-5, obj.psiMin, obj.gammaMin, obj.VMin];
                    initialCenter = (initialUpper + initialLower)/2;
                    timeStamps = 0:0.01:obj.Tf;

                    % Generate center trace
                    centerTraj = ode45(@(~,y) obj.uavDynamics(y), [0 obj.Tf], initialCenter);
                    centerTrace = deval(centerTraj,timeStamps)';
                    traceLength = length(centerTrace(:,1));
                    numStates = length(centerTrace(1,:));
                    Traces = zeros(obj.numTraces,traceLength, numStates + 1); % +1 is for the time axis
                    Traces(1,:,:) = [timeStamps', centerTrace];

                    % simulate N traces and store them in an array
                    % TODO - Vectorize this operation (a set of initialsets and vectorized interpolation)
                    for i = 2:obj.numTraces
                        initialNew = (initialUpper - initialLower).*rand(1,6) + initialLower; %Randomly generate initial set within the bound
                        iTraj = ode45(@(~,y) obj.uavDynamics(y), [0 obj.Tf], initialNew);
                        iTrace = deval(iTraj,timeStamps)';
                        Traces(i,:,:) = [timeStamps', iTrace];
                    end

                case {'fixedwing'}
                    initialUpper = [obj.x+5; obj.y+5; obj.z+5; obj.VMax]; %during execution assign x,y,z from MDP
                    initialLower = [obj.x-5; obj.y-5; obj.z-5; obj.VMin];
                    initialCenter = (initialUpper + initialLower)/2;
                    timeStamps = 0:0.01:obj.Tf;

                    model = fixedwing;

                    s = state(model);
                    s(1:4) = initialCenter;
                    u = control(model);
                    u.RollAngle = pi/12;
                    u.AirSpeed = 5;
                    e = environment(model);
                    centerTraj = ode45(@(~,x)derivative(model,x,u,e), [0 obj.Tf], s);
                    centerTrace = deval(centerTraj,timeStamps)';
                    traceLength = length(centerTrace(:,1));
                    numStates = length(centerTrace(1,:));
                    Traces = zeros(obj.numTraces,traceLength, numStates + 1); % +1 is for the time axis
                    Traces(1,:,:) = [timeStamps', centerTrace];

                    for i = 2:obj.numTraces
                        initialNew = (initialUpper - initialLower).*rand(1,4)' + initialLower; %Randomly generate initial set within the bound
                        s(1:4) = initialNew;
                        iTraj = ode45(@(~,x)derivative(model,x,u,e), [0 obj.Tf], s);
                        iTrace = deval(iTraj,timeStamps)';
                        Traces(i,:,:) = [timeStamps', iTrace];
                    end

                case {'multirotor'}
                    initialUpper = [obj.x+5; obj.y+5; obj.z+5]; %during execution assign x,y,z from MDP
                    initialLower = [obj.x-5; obj.y-5; obj.z-5];
                    initialCenter = (initialUpper + initialLower)/2;
                    timeStamps = 0:0.01:obj.Tf;

                    model = multirotor;

                    s = state(model);
                    s(1:3) = initialCenter;
                    u = control(model);
                    u.Roll = pi/12;
                    u.Thrust = 1;
                    e = environment(model);
                    centerTraj = ode45(@(~,x)derivative(model,x,u,e), [0 obj.Tf], s);
                    centerTrace = deval(centerTraj,timeStamps)';
                    traceLength = length(centerTrace(:,1));
                    numStates = length(centerTrace(1,:));
                    Traces = zeros(obj.numTraces,traceLength, numStates + 1); % +1 is for the time axis
                    Traces(1,:,:) = [timeStamps', centerTrace];

                    for i = 2:obj.numTraces
                        initialNew = (initialUpper - initialLower).*rand(1,3)' + initialLower; %Randomly generate initial set within the bound
                        s(1:3) = initialNew;
                        iTraj = ode45(@(~,x)derivative(model,x,u,e), [0 obj.Tf], s);
                        iTrace = deval(iTraj,timeStamps)';
                        Traces(i,:,:) = [timeStamps', iTrace];
                    end

                otherwise
                    error('Unrecognized vehicle type %s', vehicle)
            end


        end


        function initialRadii = computeInitialRadii(obj,Traces)
            % compute initial radii, which is the max difference between initial states among all traces
            numStates = length(Traces(1,1,:));

            initialRadii = zeros(1,numStates - 1); 
            for i = 1:obj.numTraces % Find the max difference between initial states among all traces
                trace = Traces(i,:,:);
                for j = 2:(numStates)
                    initialRadii(j - 1) = max(initialRadii(j - 1), abs(Traces(1,1,j) - trace(1,1,j)));
                end
            end
        end


        function [chebDistance] = chebyshevDistance(obj, arrayA)
            % A method that returns chebyshev distance between points given
            % in array A
            arrayB = arrayA;
            [na,aDims] = size(arrayA);
            [nb,bDims] = size(arrayB);

            [j,i] = meshgrid(1:nb,1:na);

            delta = arrayA(i,:) - arrayB(j,:);

            cheby = zeros(na,nb);

            cheby(:) = max(abs(delta),[],2);

            chebDistance = cheby(j>i);

        end


        function muPoints = computeMuPoints(obj, Traces, initialRadii)
            % A method that returns the sensitivities for each state
            % trajectory 

            numStates = length(Traces(1,1,:));
            traceLength = length(Traces(1,:,1));

            normalizingInitialRadii = repmat(initialRadii,1);
            muPoints = zeros(length(normalizingInitialRadii), traceLength-1); % exclude the initial point
            normalizingInitialRadii(find(normalizingInitialRadii==0)) = 1;

            normalizingInitialPoints = permute(Traces(:,1,2:end),[1 3 2]) ./normalizingInitialRadii;
            % Find the max distance between initial states using Chebyshev distance
            %             initialDistances = distmat(normalizingInitialPoints,'chebyshev');

            initialDistances = chebyshevDistance(obj, normalizingInitialPoints);

            for stateIndex = 2:numStates
                % Find the max distance between trajectories at each point and divide it by the max distance between initial set (aka sensitivity of trajectories)
                for timeIndex = 2:traceLength %mu_points are y_points

                    stateSensitivity =  ( chebyshevDistance(obj, reshape(Traces(:,timeIndex,stateIndex), ...
                        [obj.numTraces, 1]))./normalizingInitialRadii(stateIndex - 1))./initialDistances;
                    muPoints(stateIndex - 1, timeIndex - 1) = max(stateSensitivity(~isinf(stateSensitivity)));
                
                end
            end

        end


        function allStateLinearSepartors = computeDiscrepancyParameters(obj, Traces, initialRadii)

            % Get discrepancy parameters
            numStates = length(Traces(1,1,:));
            traceLength = length(Traces(1,:,1));
            muPoints = obj.computeMuPoints(Traces, initialRadii);

            traceInitialTime = Traces(1,1,1);
            nuPoints = Traces(1,:,1) - traceInitialTime; 
            points = zeros(numStates - 1, traceLength, 2);
            points(find(initialRadii ~= 0), 1, 2) = 1.0;
            points(:,:,1) = repmat(reshape(nuPoints, [1, length(nuPoints)])', [1,numStates-1])';
            points(:,2:end,2) = muPoints;



            points(:,:,2) = max(points(:,:,2), obj.EPSILON);
            points(:,:,2) = log(points(:,:,2)); % The log here is used to fit exponentials 
           
            for stateIndex = 2:numStates
                newMin = min(min(points(stateIndex -1, 2:end, 2)) + obj.trueMIN, -10);
                if initialRadii(stateIndex - 1) == 0
                    newPoints = vertcat([points(stateIndex - 1,2,1), ...
                        newMin], [points(stateIndex - 1,end,1), newMin]);
                else
                    newPoints = vertcat(permute( points(stateIndex - 1, 1, :), [3,2,1])' ...
                        ,[points(stateIndex - 1,1,1), newMin] , ...
                        [points(stateIndex - 1,end,1), newMin]);

                end
                statePoints = [permute( points(stateIndex - 1,2:end,:), [2,3,1]); newPoints];
                statePointsHull = sortrows(convhulln(statePoints)); 
                linearSeparators = [];
                vertInds = statePointsHull;

                for k = 1:length(vertInds)
                    endInd = vertInds(k,1);
                    startInd = vertInds(k,2);

                    if statePoints(startInd, 2) ~= newMin &&  statePoints(endInd, 2 ) ~= newMin
                        slope = (statePoints(endInd, 2) - statePoints(startInd, 2)) / ...
                            (statePoints(endInd, 1) - statePoints(startInd, 1));

                        yIntercept = statePoints(startInd, 2) - statePoints(startInd, 1)*slope;

                        startTime = statePoints(startInd, 1);
                        endTime = statePoints(endInd, 1);

                        assert  (startTime < endTime)


                        if startTime == 0
                            linearSeparators = [linearSeparators; [startTime, endTime, slope, yIntercept, 0, endInd + 1 ]];
                        else
                            linearSeparators = [linearSeparators; [startTime, endTime, slope, yIntercept, startInd + 1, endInd + 1]];
                        end

                    end
                end
                linearSeparators = sortrows(linearSeparators);
                allStateLinearSepartors{:,:,stateIndex-1} =  linearSeparators;

            end

        end

        function reachTube = getReachtube(obj, Traces, initialRadii, discrepancyParameters)
           
            % Get reach tube
            centerTrace = permute(Traces(1,:,:), [2,3,1]);

            numStates = length(centerTrace(1,:));
            traceLength = length(centerTrace(:,1));

            normalizingInitialradii = repmat(initialRadii,1);
           
            normalizingInitialradii(find(normalizingInitialradii==0)) = 1;
            df = zeros(traceLength, numStates); 
            allStateLinearSepartors = discrepancyParameters;

            for stateIndex = 2:numStates
                prevVal = 0;
                if initialRadii(stateIndex - 1) == 0
                    prevInd = 1;
                else
                    prevInd = 0;
                end
                linearSeparators = cell2mat(allStateLinearSepartors(:,:,stateIndex - 1));



                if initialRadii(stateIndex - 1) ~= 0
                    df(1, stateIndex) = initialRadii(stateIndex - 1);
                end


                for linSep = 1:length(linearSeparators(:,1))
                    linearSeparator = linearSeparators(linSep,:);
                    [~, ~, slope, yIntercept, startInd, endInd] = ...
                        deal(linearSeparator(1), linearSeparator(2), linearSeparator(3), linearSeparator(4), linearSeparator(5), linearSeparator(6));

                    segment_t = centerTrace(startInd+1:endInd, 1);
                    segment_df = normalizingInitialradii(stateIndex - 1) * exp(yIntercept + slope*segment_t);
                    segment_df(1) = max(segment_df(1), prevVal);
                    df(startInd+1:endInd, stateIndex) = segment_df;
                    prevVal = segment_df(end);
                    prevInd = endInd;

                end
            end

            reachTube = zeros(traceLength - 1, 2, numStates);
            reachTube(:,1,:) = min(centerTrace(2:end,:) - df(2:end,:), centerTrace(1:end-1,:) - df(1:end-1,:));
            reachTube(:,2,:) = max(centerTrace(2:end,:) + df(2:end,:), centerTrace(1:end-1,:) + df(1:end-1,:));

        end

    end

end


