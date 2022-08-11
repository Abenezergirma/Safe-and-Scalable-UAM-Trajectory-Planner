function [positivePeaks, negativePeaks] = buildPeaks(ownship, droneList)
% builds a set of reward sources for a certain ownship

ownX = ownship.position(1);

ownY = ownship.position(2);

ownZ = ownship.position(3);

negativePeaks = [];

for i = 1:numel(droneList) % Loop through the list of created drone objects
    drone = droneList{i};
    if drone.aircraftID == ownship.aircraftID %the case when the ownship ID and intrudenr ID is the same
        continue
    end
    if drone.dead % Skip the drone if its out of the game
        continue
    end

    [intruderX, intruderY, intruderZ] = deal(drone.position(1), drone.position(2), drone.position(3)); % intruder pos

    distance = sqrt( (ownX - intruderX)^2 + (ownY - intruderY)^2 + (ownZ - intruderZ)^2 );

    if distance < 1500
        %compute reachable set here and build the peaks inside the set

        model = DryVR(0.01,500); % [x, y, z] pos of the vehicle

        initialStates = drone.currentStates;

        Traces = GenerateTraces(model, 'fwdProjectFast', initialStates);

        initialRadii = computeInitialRadii(model,Traces);
        discrepancyParameters = computeDiscrepancyParameters(model, Traces, initialRadii);
        reachTube = getReachtube(model, Traces, initialRadii, discrepancyParameters);

        xStateBounds = reachTube(:,:,2);
        yStateBounds = reachTube(:,:,3);
        zStateBounds = reachTube(:,:,4);

        segment = 1:45:length(xStateBounds);

        Xmin = xStateBounds(segment,1);
        Xmax = xStateBounds(segment,2);
        Ymin = yStateBounds(segment,1);
        Ymax = yStateBounds(segment,2);
        Zmin = zStateBounds(segment,1);
        Zmax = zStateBounds(segment,2);
        
        xRange = (Xmax - Xmin);
        yRange = (Ymax - Ymin);
        zRange = (Zmax - Zmin);
        center = [Xmin + xRange/2, Ymin + yRange/2, Zmin + zRange/2];
        radius = sqrt(xRange.^2 + yRange.^2 + zRange.^2) * 0.5;

        negativePeaks = [negativePeaks; [repmat(1000, length(segment),1), center, repmat(0.97, length(segment),1) ,radius*1.5 ]];

    end

end

[goalX, goalY, goalZ] = deal(ownship.goal(1), ownship.goal(2), ownship.goal(3));

positivePeaks = [500, goalX, goalY, goalZ, 0.999, inf];

end