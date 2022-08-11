function [terminal, NMACs, droneList] = terminalDetection(droneList)

NMACs = 0;
NMACsPair = zeros(numel(droneList),numel(droneList));
for own = 1:numel(droneList)

    ownship = droneList{own};

    %check if the ownship is dead
    if ownship.dead
        continue
    end

    %check if the ownship violates the hit threshold
    ownship.hit = false; 

    if ownship.hit
        ownship.hitCounter = ownship.hitCounter + 1;
    end


    %check with respect to other intruders
    for intr = 1:numel(droneList)
        if own == intr %intr is the index for intruders
            continue
        end
      
        intruder = droneList{intr};
        
        if intruder.dead
            continue
        end
        %         distance = norm(ownship.currentStates(1:3) - intruder.currentStates(1:3));
        hit_distance = norm(ownship.currentStates(1:3) - intruder.currentStates(1:3));
        if hit_distance < 500 %one step filter for n_x selection

            ownship.hit = true;
            if hit_distance < 100
                % check if the case is already identified
                if NMACsPair(ownship.aircraftID, intruder.aircraftID) == true || NMACsPair( intruder.aircraftID, ownship.aircraftID) == true
                    continue
                end

                NMACs = NMACs + 1;
                NMACsPair(ownship.aircraftID, intruder.aircraftID) = true;

            end
        end

    end

    droneList{own} = ownship;
end

terminal = true;

for drone = 1:numel(droneList)
    if ~droneList{drone}.dead
        terminal = false;
    end
end

end